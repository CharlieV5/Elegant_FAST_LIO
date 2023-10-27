/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#ifndef FASTLIO_COMMON_OPTIONS_H_
#define FASTLIO_COMMON_OPTIONS_H_

#include <string>

#include "common/common_lib.h"

namespace fastlio {
namespace options {

// system part.
extern std::string opt_root_directory;
extern std::string opt_map_file_path;
extern bool   opt_enable_runtime_logging;
extern bool   opt_colorize_output_features;

inline std::string GetFilePath(const std::string& file_name) 
{ return std::string(std::string(opt_root_directory) + "Log/"+ file_name); }

// extrinsics part.
extern double opt_time_offset_lidar_to_imu;
extern V3D    opt_lidar_T_wrt_imu;
extern M3D    opt_lidar_R_wrt_imu;

// point cloud preprocess part.
extern double opt_vf_size_surf;
extern double opt_vf_size_corner;

// local map part.
extern double opt_vf_size_map;
extern double opt_map_side_len;
extern double opt_det_range;
extern double opt_move_thres;

// common part.
// extern double opt_pi_m;
// extern double opt_g_m_s2;               // Gravaty const in GuangDong/China
// extern double opt_dim_state;            // Dimension of states (Let Dim(SO(3)) = 3)
// extern double opt_dim_proc_n;           // Dimension of process noise (Let Dim(SO(3)) = 3)
// extern double opt_lidar_sp_len;         // 没搞明白这是个啥,是个正数就行？
// extern double opt_init_cov;             // 
// extern double opt_num_match_points;     // 寻找N个近邻点拟合平面(Correspondences)
// extern double opt_max_meas_dim;

// imu process part.
extern int opt_min_imu_for_init;        // 解释：IMUProc初始化最少需要几帧IMU

// ekf part.
extern double opt_ekf_wait_lidar_secs;
extern int    opt_ekf_max_iterations;
extern double opt_gyr_cov;
extern double opt_acc_cov;
extern double opt_gyr_bias_cov;
extern double opt_acc_bias_cov;
extern double opt_laser_point_cov;

// lio (mapping) part.
extern bool   opt_enable_esti_extrins;
extern bool   opt_enable_save_pcds;
extern int    opt_save_pcd_every_n;


} // namespace options
} // namespace fastlio





#endif // FASTLIO_COMMON_OPTIONS_H_