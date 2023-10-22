/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#include "common/options.h"

namespace fastlio {
namespace options {

// system part.
std::string opt_root_directory = "";
std::string opt_map_file_path = "";
bool   opt_enable_runtime_logging = false;

// extrinsics part.
double opt_time_offset_lidar_to_imu = 0.0;
V3D    opt_lidar_T_wrt_imu = Zero3d;
M3D    opt_lidar_R_wrt_imu = Eye3d;

// lidar preprocess part.       // TODO:包括适配不同型号的雷达


// point cloud preprocess part.
double opt_vf_size_surf = 0.5;
double opt_vf_size_corner = 0.5;

// local map part.
double opt_vf_size_map = 0.5;
double opt_map_side_len = 200.0;
double opt_det_range = 300.0;
double opt_move_thres = 1.5;

// common part.
// double opt_pi_m (3.14159265358);
// double opt_g_m_s2 (9.81);           // Gravaty const in GuangDong/China
// double opt_dim_state (18);          // Dimension of states (Let Dim(SO(3)) = 3)
// double opt_dim_proc_n (12);         // Dimension of process noise (Let Dim(SO(3)) = 3)
// double opt_lidar_sp_len (2);        // 没搞明白这是个啥,是个正数就行？
// double opt_init_cov   (1);          // 
// double opt_num_match_points (5);    // 寻找N个近邻点拟合平面(Correspondences)
// double opt_max_meas_dim (10000);

// imu process part.
int opt_min_imu_for_init (10);      // 解释：IMUProc初始化最少需要几帧IMU

// ekf part.
double opt_ekf_wait_lidar_secs = (0.1);
int    opt_ekf_max_iterations = 4;
double opt_gyr_cov = 0.1;
double opt_acc_cov = 0.1;
double opt_gyr_bias_cov = 0.0001;
double opt_acc_bias_cov = 0.0001;
double opt_laser_point_cov = (0.001);

// lio (mapping) part.
bool   opt_enable_esti_extrins = true;
bool   opt_enable_save_pcds = false;
int    opt_save_pcd_every_n = 100;



} // namespace options
} // namespace fastlio