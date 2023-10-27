/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#ifndef FASTLIO_UTILS_H_
#define FASTLIO_UTILS_H_


#include "common/common_lib.h"
#include "IKFoM_toolkit/use_ikfom.hpp"

#include <unordered_map>


namespace fastlio {
} // namespace fastlio



template<typename T>
inline void PointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po, const state_ikfom& state)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state.rot * (state.offset_R_L_I * p_body + state.offset_T_L_I) + state.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

inline void PointBodyToWorld(PointType const * const pi, PointType * const po, const state_ikfom& state)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state.rot * (state.offset_R_L_I * p_body + state.offset_T_L_I) + state.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

inline void PointLidarToBodyIMU(PointType const * const pi, PointType * const po, const state_ikfom& state)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state.offset_R_L_I * p_body_lidar + state.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

template<typename T>
Eigen::Matrix<T, 3, 3> EulerToMatrix(T roll, T pitch, T yaw) {
    // Eigen::Vector3d::UnitX();
    // Eigen::AngleAxisd(1, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix<T, 3, 3> rotation =
        (Eigen::AngleAxis<T>(roll, Eigen::Matrix<T, 3, 1>::UnitX()) * 
        Eigen::AngleAxis<T>(pitch, Eigen::Matrix<T, 3, 1>::UnitY()) * 
        Eigen::AngleAxis<T>(yaw, Eigen::Matrix<T, 3, 1>::UnitZ())).toRotationMatrix();
    return rotation;
}

// Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX()) * 
// Eigen::Vector3<T>
// Eigen::Matrix<T, 3, 1>::UnitX();


template<typename PT>
class VoxelDownsampler {
   public:
    VoxelDownsampler() {}
    ~VoxelDownsampler() {}

    /// @brief 0:几何中心最近点，1:均值点
    bool SetStrategy(const int& method = 0) {
        if (method < 0 || method > 1) { return false; }
        strategy_ = method;
        return true;
    }

    void setLeafSize(float lx, float ly, float lz) {
        //
    }

    // void setInputCloud(const CloudType& input) {
    //     //
    // }

    // void filter(CloudType& output) {
    //     //
    // }

    void setInputCloud(const pcl::PointCloud<PT>& input) {
        //
    }

    void filter(pcl::PointCloud<PT>& output) {
        //
    }

   private:
    float lx_ = 0.1;
    float ly_ = 0.1;
    float lz_ = 0.1;
    int strategy_ = 0;

    struct LeafType {
        int n = 0;
        PT pt_ = PT();
        Eigen::Vector4f sum_ = Eigen::Vector4f::Zero();
        Eigen::Vector4f centroid_ = Eigen::Vector4f::Zero();

        LeafType(/*3D index*/) {/*compute centroid.*/}

        void AddPoint(const PT& pt, const int method) {
            if (n == 0) { pt_ = pt; }
            if (method == 0) {
                // closest
            }
            else if (method == 1) {
                // mean
            }
        }

        // PT GetPoint() {
        //     return pt_;
        // }
    };

    std::unordered_map<size_t, LeafType> leaves_;

};



#endif // FASTLIO_UTILS_H_