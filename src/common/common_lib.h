#ifndef FASTLIO_COMMON_LIB_H_
#define FASTLIO_COMMON_LIB_H_

#include <deque>
#include <mutex>
#include <thread>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "math/so3_math.h"
#include "common/sensor_type.h"


/// NOTE: 考虑namespace是否把宏定义包进去

// 必要的宏
#define kMaxLogN (720000)   // RuntimeLogging的最大保存数量

#define USE_IKFOM           // 似乎没什么用啊，考虑转移或删掉

#define PI_M (3.14159265358)
#define G_m_s2 (9.81)           // Gravaty const in GuangDong/China
#define DIM_STATE (18)          // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_PROC_N (12)         // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN    (6.0)       // 
#define LIDAR_SP_LEN (2)        // 没搞明白这是个啥,是个正数就行？
#define INIT_COV   (1)
#define NUM_MATCH_POINTS (5)    // 寻找N个近邻点拟合平面(Correspondences)
#define MAX_MEAS_DIM (10000)

#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define CLAMP_VALUE(v,min,max)   ((v>min)?((v<max)?v:max):min)
#define ARRAY_FROM_EIGEN(mat)    mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat)  std::vector<decltype(mat)::Scalar> (mat.data(), mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name)     (std::string(std::string(ROOT_DIR) + "Log/"+ name))


/// TODO: 启用命名空间
namespace fastlio {
} // namespace fastlio


typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef std::vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;
typedef Eigen::Quaterniond QuatD;

#define MD(a,b)  Eigen::Matrix<double, (a), (b)>
#define VD(a)    Eigen::Matrix<double, (a), 1>
#define MF(a,b)  Eigen::Matrix<float, (a), (b)>
#define VF(a)    Eigen::Matrix<float, (a), 1>

// 这几个变量，在header被重复包含时会被重复定义（用const声明可以解决这个问题）
const M3D Eye3d(M3D::Identity());
const M3F Eye3f(M3F::Identity());
const V3D Zero3d(0, 0, 0);
const V3F Zero3f(0, 0, 0);

// 互斥量 & 锁
using Mutex = std::mutex;
using UniqLock = std::unique_lock<std::mutex>;
using LockGuard = std::lock_guard<std::mutex>;

// 基于IMU的Pose观测
struct ImuDrivenPose6D 
{
    double offset_time = 0; // the offset time of IMU measurement w.r.t the first lidar point
    double acc[3] = {};     // the preintegrated total acceleration (global frame) at the Lidar origin
    double gyr[3] = {};     // the unbiased angular velocity (body frame) at the Lidar origin
    double vel[3] = {};     // the preintegrated velocity (global frame) at the Lidar origin
    double pos[3] = {};     // the preintegrated position (global frame) at the Lidar origin
    double rot[9] = {};     // the preintegrated rotation (global frame) at the Lidar origin
};

/// Lidar data and imu dates for the curent process
struct MeasureGroup
{
    MeasureGroup()
    {
        lidar_beg_time = 0.0;
        this->lidar.reset(new PointCloudXYZI());
    };
    double lidar_beg_time;
    double lidar_end_time;
    PointCloudXYZI::Ptr lidar;
    std::deque<fastlio::ImuData::Ptr> imu_queue; // 去ROS
    std::deque<fastlio::WheelData::Ptr> wheel_queue; 
    std::deque<fastlio::GnssData::Ptr> gnss_queue; 
};

struct StatesGroup
{
    StatesGroup() {
		this->rot_end = M3D::Identity();
		this->pos_end = Zero3d;
        this->vel_end = Zero3d;
        this->bias_g  = Zero3d;
        this->bias_a  = Zero3d;
        this->gravity = Zero3d;
        this->cov     = MD(DIM_STATE,DIM_STATE)::Identity() * INIT_COV;
        this->cov.block<9,9>(9,9) = MD(9,9)::Identity() * 0.00001;
	};

    StatesGroup(const StatesGroup& b) {
		this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        this->cov     = b.cov;
	};

    StatesGroup& operator=(const StatesGroup& b)
	{
        this->rot_end = b.rot_end;
		this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g  = b.bias_g;
        this->bias_a  = b.bias_a;
        this->gravity = b.gravity;
        this->cov     = b.cov;
        return *this;
	};

    StatesGroup operator+(const Eigen::Matrix<double, DIM_STATE, 1> &state_add)
	{
        StatesGroup a;
		a.rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
		a.pos_end = this->pos_end + state_add.block<3,1>(3,0);
        a.vel_end = this->vel_end + state_add.block<3,1>(6,0);
        a.bias_g  = this->bias_g  + state_add.block<3,1>(9,0);
        a.bias_a  = this->bias_a  + state_add.block<3,1>(12,0);
        a.gravity = this->gravity + state_add.block<3,1>(15,0);
        a.cov     = this->cov;
		return a;
	};

    StatesGroup& operator+=(const Eigen::Matrix<double, DIM_STATE, 1> &state_add)
	{
        this->rot_end = this->rot_end * Exp(state_add(0,0), state_add(1,0), state_add(2,0));
		this->pos_end += state_add.block<3,1>(3,0);
        this->vel_end += state_add.block<3,1>(6,0);
        this->bias_g  += state_add.block<3,1>(9,0);
        this->bias_a  += state_add.block<3,1>(12,0);
        this->gravity += state_add.block<3,1>(15,0);
		return *this;
	};

    Eigen::Matrix<double, DIM_STATE, 1> operator-(const StatesGroup& b)
	{
        Eigen::Matrix<double, DIM_STATE, 1> a;
        M3D rotd(b.rot_end.transpose() * this->rot_end);
        a.block<3,1>(0,0)  = Log(rotd);
        a.block<3,1>(3,0)  = this->pos_end - b.pos_end;
        a.block<3,1>(6,0)  = this->vel_end - b.vel_end;
        a.block<3,1>(9,0)  = this->bias_g  - b.bias_g;
        a.block<3,1>(12,0) = this->bias_a  - b.bias_a;
        a.block<3,1>(15,0) = this->gravity - b.gravity;
		return a;
	};

    void resetpose()
    {
        this->rot_end = M3D::Identity();
		this->pos_end = Zero3d;
        this->vel_end = Zero3d;
    }

	M3D rot_end;        // the estimated attitude (rotation matrix) at the end lidar point
    V3D pos_end;        // the estimated position at the end lidar point (world frame)
    V3D vel_end;        // the estimated velocity at the end lidar point (world frame)
    V3D bias_g;         // gyroscope bias
    V3D bias_a;         // accelerator bias
    V3D gravity;        // the estimated gravity acceleration
    Eigen::Matrix<double, DIM_STATE, DIM_STATE>  cov;     // states covariance
};

template<typename T>
T rad2deg(T radians)
{
  return radians * 180.0 / PI_M;
}

template<typename T>
T deg2rad(T degrees)
{
  return degrees * PI_M / 180.0;
}

template<typename T>
auto ToPose6D(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g, \
              const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R)
{
    ImuDrivenPose6D rot_kp;
    rot_kp.offset_time = t;
    for (int i = 0; i < 3; i++)
    {
        rot_kp.acc[i] = a(i);
        rot_kp.gyr[i] = g(i);
        rot_kp.vel[i] = v(i);
        rot_kp.pos[i] = p(i);
        for (int j = 0; j < 3; j++) { rot_kp.rot[i*3+j] = R(i,j); }
    }
    return std::move(rot_kp);
}

inline float computeDistance(PointType p1, PointType p2)
{
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

/** ##readme##
 * plane equation: Ax + By + Cz + D = 0
 * convert to: A/D*x + B/D*y + C/D*z = -1
 * solve: A0*x0 = b0
 * where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
 * normvec:  normalized x0
*/
template<typename T>
bool estimateNorm(Eigen::Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num)
{
    Eigen::MatrixXf A(point_num, 3);
    Eigen::MatrixXf b(point_num, 1);
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_num; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }
    normvec = A.colPivHouseholderQr().solve(b);
    
    for (int j = 0; j < point_num; j++)
    {
        if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
        {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

template<typename T>
bool estimatePlane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    // 以下，解超定线性方程组【等价于最小二乘】ax+by+cz+d=0【等价于a'x+b'y+c'z=-1，其中a'=a/d，b'和c'同理】，获得{a,b,c,d}
    Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;     // a
    pca_result(1) = normvec(1) / n;     // b
    pca_result(2) = normvec(2) / n;     // c
    pca_result(3) = 1.0 / n;            // d

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        // 校验平面有效性：如果有某个点到平面的距离超过阈值（通常是0.1米），则认为不构成有效特征。
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y 
            + pca_result(2) * point[j].z + pca_result(3)) > threshold )
        {
            return false;
        }
    }
    return true;

    /** 评价：当返回true时，实质上兼容了面特征和线特征，因为无论点的空间分布如何（面或线或中间态），
     * 超定方程组都会给出一个解；而最后一步的平面有效性检验，只是排除了中间态，是允许共面以及共线的。*/
}


#endif // FASTLIO_COMMON_LIB_H_