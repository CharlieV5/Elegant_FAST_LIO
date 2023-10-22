/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#ifndef FASTLIO_UTILS_H_
#define FASTLIO_UTILS_H_


#include "common/common_lib.h"
#include "IKFoM_toolkit/use_ikfom.hpp"


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




#endif // FASTLIO_UTILS_H_