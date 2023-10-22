
#ifndef FASTLIO_IMU_PROCESS_H_
#define FASTLIO_IMU_PROCESS_H_

#include <deque>
#include <cmath>
#include <mutex>
#include <thread>
#include <fstream>
#include <cassert>

#include <omp.h>

#include <Eigen/Eigen>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "math/so3_math.h"
#include "common/common_lib.h"
#include "IKFoM_toolkit/use_ikfom.hpp"
#include "common/options.h"


/// ************* IMU Process and undistortion
class ImuProcess {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  // void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  // void Reset(double start_timestamp, const fastlio::ImuData::Ptr &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;

  /// 唯一接口
  bool Process(const MeasureGroup &meas,  IeskfomType &kf_state, PointCloudXYZI::Ptr pcl_un_);

  V3D cov_acc_esti;   // 估算出来的协方差
  V3D cov_gyro_esti;  // 估算出来的协方差
  V3D cov_acc_fixed;  // 人为设定的协方差
  V3D cov_gyro_fixed; // 人为设定的协方差
  V3D cov_bias_gyro;
  V3D cov_bias_acc;
  double first_lidar_time;
  std::ofstream fout_imu;

 private:
  /// 下边这两个是核心函数
  bool InitWithIMUs(const MeasureGroup &meas, IeskfomType &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, IeskfomType &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  fastlio::ImuData::Ptr last_imu_obs_;
  std::vector<ImuDrivenPose6D> imu_poses_;
  std::vector<M3D>    v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D gyro_last;  // 上一帧gyro观测
  V3D acc_s_last; // 上一帧acc观测(注意是unbiased,转换到world系,并且抵消了重力的纯净的acc)
  double start_timestamp_;
  double last_lidar_end_time_;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
  int    init_imu_num_ = 1;      // 原名[init_iter_num]，从使用情况来看，这个变量实际指的是imu的数量，因此改名之
};


#endif // FASTLIO_IMU_PROCESS_H_
