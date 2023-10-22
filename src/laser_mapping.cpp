/**
 * This is an advanced implementation of the algorithm described in the
 * following paper:
 *   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
 *     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
 * 
 * Modifier: Livox               dev@livoxtech.com
 * 
 * Copyright 2013, Ji Zhang, Carnegie Mellon University
 * Further contributions copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*/

#include <mutex>
#include <cmath>
#include <thread>
#include <fstream>
#include <csignal>
#include <condition_variable>

#include <omp.h>
#include <unistd.h>
#include <Python.h>

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include "math/so3_math.h"
#include "common/options.h"
#include "common/utils.h"
#include "core/imu_process.h"
#include "ikd-Tree/ikd_Tree.h"
#include "fast_lio.h"

// ROS部分
#include "msg_conversion.h"
#include "lidar_preprocess.h"

// ROS部分
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <livox_ros_driver/CustomMsg.h>


// ====================== ROS组件 ======================
nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::PoseStamped msg_body_pose;

ros::Publisher pubLaserCloudRegistered;
ros::Publisher pubLaserCloudRegisInBody;
ros::Publisher pubLaserCloudMatchedSurfs;
ros::Publisher pubIkdtreeMap;
ros::Publisher pubOdomAftMapped;
ros::Publisher pubLioPath;

bool   enable_pub_scan = false, enable_pub_dense = false;
bool   enable_pub_body_scan = false;
bool   enable_pub_path = true;

int    publish_count = 0;
int    kPubFramePeriod = (20);

bool   ros_enable_time_sync = false;
std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);

std::string lidar_topic_name_, imu_topic_name_;
std::string kWorldFrameId = "map";
std::string kBodyFrameId = "body";

bool   flag_lidar_pushed = false, flag_exit = false;

// 线程
std::mutex mapping_mtx_;                 // 在三个Handle**回调里加了这个锁，推测应该是用来保护三个buffer的？
std::condition_variable condition_var_;  // 在三个Handle**回调中添加完数据后触发这个，推测应该是触发检查measures是否凑齐，若凑齐则触发EKF处理。

// 缓存
std::deque<double>                     lidar_time_buffer;
std::deque<PointCloudXYZI::Ptr>        lidar_buffer;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

// 传感器消息 & 预处理
MeasureGroup measures_;
double last_timestamp_lidar = 0, last_timestamp_Imu = -1.0;
std::shared_ptr<LidarPreprocess> lidar_processor_(new LidarPreprocess()); /*ros coupled*/


// ====================== 非ROS部分 ======================

// 保存地图
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
int    pcd_file_id = 0;

// Time Log Variables
int    frame_num = 0;
int    logging_count = 0;
double avg_time_consu = 0, avg_time_icp = 0, avg_time_match = 0;
double avg_time_incre = 0, avg_time_solve = 0, avg_time_const_H_time = 0;

double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[kMaxLogN], s_plot[kMaxLogN], s_plot2[kMaxLogN], s_plot3[kMaxLogN], s_plot4[kMaxLogN];
double s_plot5[kMaxLogN], s_plot6[kMaxLogN], s_plot7[kMaxLogN], s_plot8[kMaxLogN];
double s_plot9[kMaxLogN], s_plot10[kMaxLogN], s_plot11[kMaxLogN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, num_added_points = 0, num_deleted_points = 0; /*kdtree_delete_counter*/

// 逻辑：类内变量
bool   flag_first_scan = true, flag_SCAN_inited = false;
double G_first_lidar_time = 0.0, G_lidar_end_time = 0, G_total_distance = 0;
int    received_scan_count = 0;

int    iterCount = 0, laserCloudValidNum = 0;
double residual_mean_last = 0.05, total_residual = 0.0;

// 预处理
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

// 特征点云
int    G_num_dsampled_features = 0; // 预处理后的scan降采样后才能给到算法用
int    G_num_matched_features = 0;  // 一个观测单元中，所有match成功的feature的数量 
PointCloudXYZI::Ptr laser_features_deskewed_(new PointCloudXYZI());   // 预处理(会跳点!)后经运动补偿且转移到IMU系的所有点
PointCloudXYZI::Ptr features_dsampled_in_body_(new PointCloudXYZI()); // 体素降采样后的点，也是算法真正用的点
PointCloudXYZI::Ptr features_dsampled_in_world_(new PointCloudXYZI());// 和body系下的点一一对应，会被酌情添加到ikdtree地图中

// 是否关联到了有效平面(也即match)
std::vector<PointVector>  features_nearest_neighbors;    // 保存每个feature点的NN近邻点
bool features_matched_info[100000] = {0};           // 各个feature是否成功match（在kdtree中的近邻形成了有效平面）的标签
float features_residuals_info[100000] = {0.0};      // 所有match点的点面距离残差
PointCloudXYZI::Ptr features_norms_info(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laser_features_matched_(new PointCloudXYZI(100000, 1)); // 成功match的特征点
PointCloudXYZI::Ptr matched_norms_(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr features_arrayed_; // unused, 存储从ikdtree中remove掉的点，大概是用于debug

// ikdtree相关
bool visulize_ikdtree_points = false;
bool ikdtree_points_updated = false;
PointCloudXYZI::Ptr points_from_ikdtree_(new PointCloudXYZI()); // ikdtree点云地图可视化
std::vector<ikdtreeNS::BoxPointType> boxes_need_remove;     // 每次trim localmap时，需要被裁剪掉的区域

// 未启用
V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);

// 算法设施
ikdtreeNS::KD_TREE<PointType> iKdTree;
std::shared_ptr<ImuProcess> imu_processor_(new ImuProcess());

// EKF相关
IeskfomType ieskf_estimator_;
state_ikfom current_state;
vect3 current_lidar_posi;
V3D current_euler;

// 其它
FILE *fp_lio_state;
std::ofstream fout_pre, fout_out, fout_dbg;

// *************** Algorithm funcs ***************

inline void DumpLioStateToLog(FILE *fp, const state_ikfom& state, const MeasureGroup& measures)  
{
    V3D rot_ang(Log(state.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", measures.lidar_beg_time - G_first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state.pos(0), state.pos(1), state.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state.vel(0), state.vel(1), state.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state.bg(0), state.bg(1), state.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state.ba(0), state.ba(1), state.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state.grav[0], state.grav[1], state.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

// 该函数提供一个指定接口的计算xxx的范式(作为一个函数对象被传递到EKF滤波器中)，在EKF中作为{观测模型}使用
void h_share_model(state_ikfom &ikfom_state, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laser_features_matched_->clear(); 
    matched_norms_->clear(); 
    total_residual = 0.0; 

    // 核心：为每个surf-feature点寻找近邻，拟合平面，计算点到面残差
    #ifdef MP_EN /*支持OMP并行加速*/
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < G_num_dsampled_features; i++)
    {
        PointType &point_body  = features_dsampled_in_body_->points[i]; 
        PointType &point_world = features_dsampled_in_world_->points[i]; 

        // 将feature转换到world坐标系
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(ikfom_state.rot * (ikfom_state.offset_R_L_I * p_body + ikfom_state.offset_T_L_I) + ikfom_state.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        // 为feature寻找和记录近邻点
        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto& neighbor_points = features_nearest_neighbors[i];
        if (ekfom_data.converge)
        {
            iKdTree.Nearest_Search(point_world, NUM_MATCH_POINTS, neighbor_points, pointSearchSqDis);
            features_matched_info[i] = neighbor_points.size() < NUM_MATCH_POINTS ? false 
                                        : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!features_matched_info[i]) continue;

        // 用feature的近邻点拟合平面，评估平面拟合质量，并计算点到面误差【严格的match标准】
        VF(4) pabcd;
        features_matched_info[i] = false;
        if (estimatePlane(pabcd, neighbor_points, 0.1f)) {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9) {
                features_matched_info[i] = true;
                features_norms_info->points[i].x = pabcd(0);
                features_norms_info->points[i].y = pabcd(1);
                features_norms_info->points[i].z = pabcd(2);
                features_norms_info->points[i].intensity = pd2; // intensity用来保存截距
                features_residuals_info[i] = abs(pd2);
            }
        }
    }

    // 记录match成功的点
    G_num_matched_features = 0;
    for (int i = 0; i < G_num_dsampled_features; i++) {
        if (features_matched_info[i]) {
            laser_features_matched_->points[G_num_matched_features] = features_dsampled_in_body_->points[i];
            matched_norms_->points[G_num_matched_features] = features_norms_info->points[i];
            total_residual += features_residuals_info[i];
            G_num_matched_features ++;
        }
    }

    if (G_num_matched_features < 1)
    {
        ekfom_data.valid = false;
        std::cout << "No Matched Points! --- Failed to execute H_Share_Model." << std::endl;
        return;
    }

    residual_mean_last = total_residual / G_num_matched_features;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();

    // 计算雅可比矩阵（h_x）和残差向量（h）
    // Computation of Measuremnt Jacobian matrix H and Measurents Vector
    ekfom_data.h_x = Eigen::MatrixXd::Zero(G_num_matched_features, 12); //23
    ekfom_data.h.resize(G_num_matched_features);

    // 只有match成功的点，才会进入观测模型！【somehow,这保证了配准的质量和FastLIO的精度】
    for (int i = 0; i < G_num_matched_features; i++)
    {
        const PointType &laser_p  = laser_features_matched_->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = ikfom_state.offset_R_L_I * point_this_be + ikfom_state.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal Vector of closest surface ***/
        const PointType &norm_p = matched_norms_->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(ikfom_state.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (fastlio::options::opt_enable_esti_extrins)
        {
            V3D B(point_be_crossmat * ikfom_state.offset_R_L_I.conjugate() * C); //ikfom_state.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

void UpdateIkdtreeMap()
{
    PointVector PointsToAdd;
    PointVector PointsNoNeedDownSample;
    PointsToAdd.reserve(G_num_dsampled_features);
    PointsNoNeedDownSample.reserve(G_num_dsampled_features);
    for (int i = 0; i < G_num_dsampled_features; i++)
    {
        // 将单点转换到world系下
        PointBodyToWorld(&(features_dsampled_in_body_->points[i]), 
            &(features_dsampled_in_world_->points[i]), current_state);
        // 如果一个feature能在kdtree中找到近邻，则酌情决定是否把这个feature添加到kdtree中去
        if (!features_nearest_neighbors[i].empty() && flag_SCAN_inited) {
            const PointVector& neighbor_points = features_nearest_neighbors[i];
            bool need_add = true;
            ikdtreeNS::BoxPointType Box_of_Point; //unused
            PointType downsample_result; //unused
            PointType box_center; 
            double kIkdtreeVfSize = fastlio::options::opt_vf_size_map;
            box_center.x = floor(features_dsampled_in_world_->points[i].x / kIkdtreeVfSize) * kIkdtreeVfSize + 0.5 * kIkdtreeVfSize;
            box_center.y = floor(features_dsampled_in_world_->points[i].y / kIkdtreeVfSize) * kIkdtreeVfSize + 0.5 * kIkdtreeVfSize;
            box_center.z = floor(features_dsampled_in_world_->points[i].z / kIkdtreeVfSize) * kIkdtreeVfSize + 0.5 * kIkdtreeVfSize;
            float dist  = computeDistance(features_dsampled_in_world_->points[i], box_center);
            // 如果feature的最近邻都不在feature所在box中，证明一定要添加这个feature
            if (fabs(neighbor_points[0].x - box_center.x) > 0.5 * kIkdtreeVfSize 
                && fabs(neighbor_points[0].y - box_center.y) > 0.5 * kIkdtreeVfSize 
                && fabs(neighbor_points[0].z - box_center.z) > 0.5 * kIkdtreeVfSize ) {
                PointsNoNeedDownSample.push_back(features_dsampled_in_world_->points[i]);
                continue;
            }
            // 如果存在某个近邻，比当前feature更接近box中心，则没必要添加这个feature
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++) {
                if (neighbor_points.size() < NUM_MATCH_POINTS) break;
                if (computeDistance(neighbor_points[readd_i], box_center) < dist) {
                    need_add = false;
                    break;
                }
            }
            // 否则，意味着当前feature是最接近box中心的，有新增的必要
            if (need_add) {
                PointsToAdd.push_back(features_dsampled_in_world_->points[i]);
            }
        }
        // 如果一个feature能在kdtree中根本找不到近邻 —— 那当然要无条件新增这个feature点到kdtree中去
        else {
            PointsToAdd.push_back(features_dsampled_in_world_->points[i]);
        }
    }

    double kdtree_start_time_point = omp_get_wtime();
    num_added_points = iKdTree.Add_Points(PointsToAdd, /*downsample=*/true);
    iKdTree.Add_Points(PointsNoNeedDownSample, /*downsample=*/false); 
    num_added_points = PointsToAdd.size() + PointsNoNeedDownSample.size();
    kdtree_incremental_time = omp_get_wtime() - kdtree_start_time_point;
}

void PointsCacheCollect()
{
    // 除非调试，无必要启用
    // PointVector points_history;
    // iKdTree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) features_arrayed_->push_back(points_history[i]);
}

void TrimLocalMapAdaptively()
{
    const double kMapSideLen = fastlio::options::opt_map_side_len;
    const double kDetRange = fastlio::options::opt_det_range;
    const double kMovThreshold = fastlio::options::opt_move_thres;

    boxes_need_remove.clear();
    num_deleted_points = 0;
    kdtree_delete_time = 0.0;
    PointBodyToWorld(XAxisPoint_body, XAxisPoint_world, current_state); //也没人用啊，估计是debug留下的无用变量
    const V3D CurrLidarPosi = current_lidar_posi;
    static ikdtreeNS::BoxPointType current_map_border; // 静态变量
    static bool map_border_inited = false;  // 静态变量
    if (!map_border_inited){
        for (int i = 0; i < 3; i++){
            current_map_border.vertex_min[i] = CurrLidarPosi(i) - kMapSideLen / 2.0;
            current_map_border.vertex_max[i] = CurrLidarPosi(i) + kMapSideLen / 2.0;
        }
        map_border_inited = true;
        return;
    }

    // 常规trim流程
    float distance_to_border[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        distance_to_border[i][0] = fabs(CurrLidarPosi(i) - current_map_border.vertex_min[i]); //到下界距离
        distance_to_border[i][1] = fabs(CurrLidarPosi(i) - current_map_border.vertex_max[i]); //到上界距离
        if (distance_to_border[i][0] <= kMovThreshold * kDetRange || 
            distance_to_border[i][1] <= kMovThreshold * kDetRange) {
            need_move = true;
        }
    }
    if (!need_move) { return; }

    ikdtreeNS::BoxPointType map_border_to_reach, box_to_remove;
    map_border_to_reach = current_map_border;
    float MoveStep = std::max(
        (kMapSideLen - 2.0 * kMovThreshold * kDetRange) * 0.5 * 0.9, 
        double(kDetRange * (kMovThreshold -1))); //这貌似是个经验值
    for (int i = 0; i < 3; i++) /*轮询三个维度*/ {
        box_to_remove = current_map_border;
        if (distance_to_border[i][0] <= kMovThreshold * kDetRange) {
            // 如果到下界的距离过近，证明需要裁剪掉上界附近的区域
            box_to_remove.vertex_min[i] = current_map_border.vertex_max[i] - MoveStep;
            boxes_need_remove.push_back(box_to_remove);
            map_border_to_reach.vertex_max[i] -= MoveStep;
            map_border_to_reach.vertex_min[i] -= MoveStep;
        } else if (distance_to_border[i][1] <= kMovThreshold * kDetRange) {
            // 如果到上界的距离过近，证明需要裁剪掉下界附近的区域
            box_to_remove.vertex_max[i] = current_map_border.vertex_min[i] + MoveStep;
            boxes_need_remove.push_back(box_to_remove);
            map_border_to_reach.vertex_max[i] += MoveStep;
            map_border_to_reach.vertex_min[i] += MoveStep;
        }
    }
    current_map_border = map_border_to_reach;
    std::cout << "going to remove boxes : " << boxes_need_remove.size() << std::endl;
    std::cout << "current lidar position: {" << CurrLidarPosi.transpose() << "}" << std::endl;
    std::cout << "map border is supposed to reach: " << "{x, y, z ~ x, y, z}" << std::endl;

    PointsCacheCollect();
    double time_delete_begin = omp_get_wtime();
    if(boxes_need_remove.size() > 0) {
        num_deleted_points = iKdTree.Delete_Point_Boxes(boxes_need_remove);
    }
    kdtree_delete_time = omp_get_wtime() - time_delete_begin;
}

// *************** ROS callback ***************

void HandleCommonPointCloudMsg(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mapping_mtx_.lock();

    /// 计数计时
    received_scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    /// 预处理和入buffer，没有多余的动作
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    lidar_processor_->Process(msg, ptr);
    lidar_buffer.push_back(ptr);
    lidar_time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[received_scan_count] = omp_get_wtime() - preprocess_start_time;
    mapping_mtx_.unlock();
    condition_var_.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void HandleLivoxPointCloudMsg(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    mapping_mtx_.lock();
    double preprocess_start_time = omp_get_wtime();
    received_scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    /// 推测这种时间补偿，应该是仅存在于livox设备上的一个问题？
    if (!ros_enable_time_sync && abs(last_timestamp_Imu - last_timestamp_lidar) > 10.0 
        && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",
            last_timestamp_Imu, last_timestamp_lidar);
    }

    if (ros_enable_time_sync && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_Imu) > 1 
        && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_Imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    /// 预处理和入buffer，没有多余的动作
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    lidar_processor_->Process(msg, ptr);
    lidar_buffer.push_back(ptr);
    lidar_time_buffer.push_back(last_timestamp_lidar);
    s_plot11[received_scan_count] = omp_get_wtime() - preprocess_start_time;
    mapping_mtx_.unlock();
    condition_var_.notify_all();
}

void HandleAT128PointCloudMsg()
{
    // TODO
}

void HandleImuMsg(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // std::cout << "IMU got at: " << msg_in->header.stamp.toSec() << std::endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    /// 时间延迟补偿
    //这个误差为啥补偿到IMU上，通常IMU实时性更好，不是lidar容易时延么？
    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - fastlio::options::opt_time_offset_lidar_to_imu); 
    if (abs(timediff_lidar_wrt_imu) > 0.1 && ros_enable_time_sync)
    {
        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }
    double timestamp = msg->header.stamp.toSec();

    /// 塞到buffer里
    mapping_mtx_.lock();
    if (timestamp < last_timestamp_Imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }
    last_timestamp_Imu = timestamp;
    imu_buffer.push_back(msg);
    mapping_mtx_.unlock();
    condition_var_.notify_all();
}

void HandleSignal(int sig)
{
    flag_exit = true;
    ROS_WARN("catch sig %d", sig);
    condition_var_.notify_all();
}

bool SyncPackages(MeasureGroup &measurements)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /// push a lidar scan
    // 这里会更新lidar_end_time，这个时间不能太接近
    static double mean_per_scan_time = 0.0;
    static int    num_scans_ = 0;
    if(!flag_lidar_pushed) {
        measurements.lidar = lidar_buffer.front();
        measurements.lidar_beg_time = lidar_time_buffer.front();
        if (measurements.lidar->points.size() <= 1) {
            G_lidar_end_time = measurements.lidar_beg_time + mean_per_scan_time;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (measurements.lidar->points.back().curvature / double(1000) < 0.5 * mean_per_scan_time) {
            ROS_WARN("Lidar scan duration is shorter than expected.\n");
            G_lidar_end_time = measurements.lidar_beg_time + mean_per_scan_time;
        }
        else {
            num_scans_++;
            G_lidar_end_time = measurements.lidar_beg_time + measurements.lidar->points.back().curvature / double(1000);
            mean_per_scan_time += (measurements.lidar->points.back().curvature / double(1000) - mean_per_scan_time) / num_scans_;
        }

        measurements.lidar_end_time = G_lidar_end_time;
        flag_lidar_pushed = true;
    }

    /// 要求imu时段必须完全覆盖lidar scan时段
    if (last_timestamp_Imu < G_lidar_end_time) {
        return false;
    }

    /// push imu data, and pop from imu buffer
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    // measurements.imu.clear();
    measurements.imu_queue.clear();
    while ((!imu_buffer.empty()) && (imu_time < G_lidar_end_time)) {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > G_lidar_end_time) { break; }
        // measurements.imu.push_back(imu_buffer.front());
        measurements.imu_queue.push_back(ToImuData(imu_buffer.front()));
        imu_buffer.pop_front();
    }

    /// pop from lidar buffer, then reset.
    lidar_buffer.pop_front();
    lidar_time_buffer.pop_front();
    flag_lidar_pushed = false;
    return true;
}

// *************** ROS Pub ***************

void PublishKfPointsInWorld(const ros::Publisher & pubLaserCloudRegistered)
{
    if(enable_pub_scan)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(enable_pub_dense ? 
            laser_features_deskewed_ : features_dsampled_in_body_);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++) {
            PointBodyToWorld(&laserCloudFullRes->points[i], 
                &laserCloudWorld->points[i], current_state);
        }
        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(*laserCloudWorld, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(G_lidar_end_time);
        laser_cloud_msg.header.frame_id = kWorldFrameId;
        pubLaserCloudRegistered.publish(laser_cloud_msg);
        publish_count -= kPubFramePeriod;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (fastlio::options::opt_enable_save_pcds)
    {
        int size = laser_features_deskewed_->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            PointBodyToWorld(&laser_features_deskewed_->points[i], 
                &laserCloudWorld->points[i], current_state);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && fastlio::options::opt_save_pcd_every_n > 0
            && scan_wait_num >= fastlio::options::opt_save_pcd_every_n)
        {
            pcd_file_id ++;
            std::string pcd_file_path(std::string(fastlio::options::opt_root_directory + "PCD/scans_") + 
                                        std::to_string(pcd_file_id) + std::string(".pcd"));
            std::cout << "going to save pcd file: " << pcd_file_path << std::endl;
            pcl::PCDWriter pcd_writer;
            pcd_writer.writeBinary(pcd_file_path, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void PublishKfPointsInBody(const ros::Publisher & pubLaserCloudRegisInBody)
{
    int size = laser_features_deskewed_->points.size();
    PointCloudXYZI::Ptr point_cloud_imu_body(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++) {
        PointLidarToBodyIMU(&laser_features_deskewed_->points[i], 
            &point_cloud_imu_body->points[i], current_state);
    }
    sensor_msgs::PointCloud2 laser_cloud_msg;
    pcl::toROSMsg(*point_cloud_imu_body, laser_cloud_msg);
    laser_cloud_msg.header.stamp = ros::Time().fromSec(G_lidar_end_time);
    laser_cloud_msg.header.frame_id = kBodyFrameId;
    pubLaserCloudRegisInBody.publish(laser_cloud_msg);
    publish_count -= kPubFramePeriod;
}

void PublishPointsMatchedSurfs(const ros::Publisher & pubLaserCloudMatchedSurfs)
{
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(G_num_matched_features, 1));
    for (int i = 0; i < G_num_matched_features; i++) {
        PointBodyToWorld(&laser_features_matched_->points[i], 
            &laserCloudWorld->points[i], current_state);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(G_lidar_end_time);
    laserCloudFullRes3.header.frame_id = kWorldFrameId;
    pubLaserCloudMatchedSurfs.publish(laserCloudFullRes3);
}

void PublishIkdtreeMap(const ros::Publisher & pubIkdtreeMap)
{
    if (pubIkdtreeMap.getNumSubscribers() > 0) 
    {
        visulize_ikdtree_points = true;
        if (!ikdtree_points_updated) 
        { 
            std::cout << "ikdtree points not updated, failed to publish ikdtree map ..." << std::endl;
            return; 
        }
        sensor_msgs::PointCloud2 laserCloudMap;
        pcl::toROSMsg(*points_from_ikdtree_, laserCloudMap);
        laserCloudMap.header.stamp = ros::Time().fromSec(G_lidar_end_time);
        laserCloudMap.header.frame_id = kWorldFrameId;
        pubIkdtreeMap.publish(laserCloudMap);
        ikdtree_points_updated = false;
    } 
    else 
    {
        visulize_ikdtree_points = false;
    }

}

template<typename T>
inline void SetRosPose(T & out, const state_ikfom& state)
{
    out.pose.position.x = state.pos(0);
    out.pose.position.y = state.pos(1);
    out.pose.position.z = state.pos(2);

    QuatD current_quat = QuatD::Identity();
    current_quat.x() = state.rot.coeffs()[0];
    current_quat.y() = state.rot.coeffs()[1];
    current_quat.z() = state.rot.coeffs()[2];
    current_quat.w() = state.rot.coeffs()[3];
    out.pose.orientation.x = current_quat.x();
    out.pose.orientation.y = current_quat.y();
    out.pose.orientation.z = current_quat.z();
    out.pose.orientation.w = current_quat.w();
    
}

void PublishLidarOdometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = kWorldFrameId;
    odomAftMapped.child_frame_id = kBodyFrameId;
    odomAftMapped.header.stamp = ros::Time().fromSec(G_lidar_end_time);// ros::Time().fromSec(G_lidar_end_time);
    SetRosPose(odomAftMapped.pose, current_state);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = ieskf_estimator_.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, kWorldFrameId, kBodyFrameId ) );
}

void PublishLioPath(const ros::Publisher pubLioPath)
{
    SetRosPose(msg_body_pose, current_state);
    msg_body_pose.header.stamp = ros::Time().fromSec(G_lidar_end_time);
    msg_body_pose.header.frame_id = kWorldFrameId;

    path.header.stamp    = ros::Time::now();
    path.header.frame_id = kWorldFrameId;

    /*** if path is too large, the rviz will crash ***/
    static int all_count = 0;
    all_count++;
    if (all_count % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubLioPath.publish(path);
    }
}

// *************** main ***************

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    // ros part.
    nh.param<std::string>("common/lid_topic",lidar_topic_name_,"/livox/lidar");
    nh.param<std::string>("common/imu_topic", imu_topic_name_,"/livox/imu");
    nh.param<bool>("publish/path_en",enable_pub_path, true);
    nh.param<bool>("publish/scan_publish_en",enable_pub_scan, true);
    nh.param<bool>("publish/dense_publish_en",enable_pub_dense, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",enable_pub_body_scan, true);

    nh.param<bool>("common/time_sync_en", ros_enable_time_sync, false);

    // ros (lidar preprocess) part.
    nh.param<double>("preprocess/blind", lidar_processor_->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", lidar_processor_->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", lidar_processor_->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", lidar_processor_->time_unit, US);
    nh.param<int>("preprocess/scan_rate", lidar_processor_->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", lidar_processor_->keep_every_n_points, 2);
    nh.param<bool>("feature_extract_enable", lidar_processor_->feature_enabled, false);
    std::cout << "lidar_processor_->lidar_type     : " << ToLidarString(lidar_processor_->lidar_type) << std::endl;
    std::cout << "lidar_processor_->feature_enabled: " << (lidar_processor_->feature_enabled) << std::endl;

    // system part.
    nh.param<std::string>("map_file_path",fastlio::options::opt_map_file_path,"");
    nh.param<bool>("runtime_pos_log_enable", fastlio::options::opt_enable_runtime_logging, 0);

    // implementation part.
    nh.param<double>("common/time_offset_lidar_to_imu", fastlio::options::opt_time_offset_lidar_to_imu, 0.0);
    nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
    nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());
    fastlio::options::opt_lidar_T_wrt_imu << VEC_FROM_ARRAY(extrinT);
    fastlio::options::opt_lidar_R_wrt_imu << MAT_FROM_ARRAY(extrinR);

    nh.param<double>("filter_size_corner",fastlio::options::opt_vf_size_corner,0.5); // unused
    nh.param<double>("filter_size_surf",fastlio::options::opt_vf_size_surf,0.5);

    nh.param<double>("filter_size_map",fastlio::options::opt_vf_size_map,0.5);
    nh.param<double>("cube_side_length",fastlio::options::opt_map_side_len,200);
    nh.param<double>("mapping/det_range",fastlio::options::opt_det_range,300.f);

    nh.param<int>("max_iteration", fastlio::options::opt_ekf_max_iterations, 4);
    nh.param<double>("mapping/gyr_cov", fastlio::options::opt_gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", fastlio::options::opt_acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", fastlio::options::opt_gyr_bias_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", fastlio::options::opt_acc_bias_cov, 0.0001);
    
    nh.param<bool>("mapping/extrinsic_est_en", fastlio::options::opt_enable_esti_extrins, true);
    nh.param<bool>("pcd_save/pcd_save_en", fastlio::options::opt_enable_save_pcds, false);
    nh.param<int>("pcd_save/interval", fastlio::options::opt_save_pcd_every_n, 100);

    fastlio::options::opt_root_directory = ROOT_DIR;

    const double kVFSizeSurf = fastlio::options::opt_vf_size_surf;
    const double kVFSizeMap = fastlio::options::opt_vf_size_map;
    downSizeFilterSurf.setLeafSize(kVFSizeSurf, kVFSizeSurf, kVFSizeSurf);
    downSizeFilterMap.setLeafSize(kVFSizeMap, kVFSizeMap, kVFSizeMap);

    features_arrayed_.reset(new PointCloudXYZI());
    memset(features_matched_info, true, sizeof(features_matched_info));
    memset(features_residuals_info, -1000.0f, sizeof(features_residuals_info));

    imu_processor_->set_extrinsic(fastlio::options::opt_lidar_T_wrt_imu, 
                                  fastlio::options::opt_lidar_R_wrt_imu);
    const double kGyrCov = fastlio::options::opt_gyr_cov;
    const double kAccCov = fastlio::options::opt_acc_cov;
    const double kGyrBiasCov = fastlio::options::opt_gyr_bias_cov;
    const double kAccBiasCov = fastlio::options::opt_acc_bias_cov;
    imu_processor_->set_gyr_cov(V3D(kGyrCov, kGyrCov, kGyrCov));
    imu_processor_->set_acc_cov(V3D(kAccCov, kAccCov, kAccCov));
    imu_processor_->set_gyr_bias_cov(V3D(kGyrBiasCov, kGyrBiasCov, kGyrBiasCov));
    imu_processor_->set_acc_bias_cov(V3D(kAccBiasCov, kAccBiasCov, kAccBiasCov));

    double epsi[23] = {0.001};
    std::fill(epsi, epsi+23, 0.001);
    ieskf_estimator_.init_dyn_share(get_f, df_dx, df_dw, h_share_model, 
                                    fastlio::options::opt_ekf_max_iterations, epsi);

    // debug recorder
    std::string pos_log_dir = fastlio::options::opt_root_directory + "/Log/pos_log.txt";
    fp_lio_state = fopen(pos_log_dir.c_str(),"w");
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),std::ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),std::ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),std::ios::out);
    if (fout_pre && fout_out) {
        std::cout << "~~~~" << fastlio::options::opt_root_directory << " file opened" << std::endl;
    } else {
        std::cout << "~~~~" << fastlio::options::opt_root_directory << " doesn't exist" << std::endl;
    }


    // subscribers
    ros::Subscriber sub_pcl = lidar_processor_->lidar_type == AVIA ? \
        nh.subscribe(lidar_topic_name_, 200000, HandleLivoxPointCloudMsg) : \
        nh.subscribe(lidar_topic_name_, 200000, HandleCommonPointCloudMsg);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic_name_, 200000, HandleImuMsg);

    // publishers
    pubLaserCloudRegistered = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    pubLaserCloudRegisInBody = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
    pubLaserCloudMatchedSurfs = nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_surfs", 100000);
    pubIkdtreeMap = nh.advertise<sensor_msgs::PointCloud2>("/ikdtree_map", 100000);
    pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/lio_odometry", 100000);
    pubLioPath = nh.advertise<nav_msgs::Path>("/lio_path", 100000);

    signal(SIGINT, HandleSignal);
    ros::Rate rate(5000);
    bool ros_ok = ros::ok();
    while (ros_ok)
    {
        if (flag_exit) { break; }
        ros::spinOnce();
        if(SyncPackages(measures_)) 
        {
            /// 无条件跳过第一帧？(IMUProc初始化也不需要这个,有点奇怪)
            if (flag_first_scan)
            {
                G_first_lidar_time = measures_.lidar_beg_time;
                imu_processor_->first_lidar_time = G_first_lidar_time;
                flag_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            /// 滤波器状态预测 + 运动补偿(同时也是把点云从lidar坐标系下转移到IMU坐标系下)
            // 注意这里，意味着IMUProc即使未初始化成功，也会往下进行
            /*讨论：看了这个函数体，不存在IMU(预)积分更新滤波器预测的情形，这，，，没预测啊。。。*/
            const bool inited = imu_processor_->Process(measures_, ieskf_estimator_, laser_features_deskewed_ /*output*/);

            /// 获取滤波器状态预测
            current_state = ieskf_estimator_.get_x();
            current_lidar_posi = current_state.pos + current_state.rot * current_state.offset_T_L_I;

            if (laser_features_deskewed_->empty() || (laser_features_deskewed_ == NULL))
            {
                std::cout << "No point, skip this scan!" << std::endl;
                continue;
            }

            /// 这是个偷懒的的行为(并不代表事实上是否真的初始化了)
            flag_SCAN_inited = 
                (measures_.lidar_beg_time - G_first_lidar_time) < fastlio::options::opt_ekf_wait_lidar_secs 
                ? false : true;

            /// 动态裁剪ikdtree持有的localMap。
            TrimLocalMapAdaptively();

            /// scan预处理
            downSizeFilterSurf.setInputCloud(laser_features_deskewed_);
            downSizeFilterSurf.filter(*features_dsampled_in_body_);
            t1 = omp_get_wtime(); /*downsamp耗时统计*/
            G_num_dsampled_features = features_dsampled_in_body_->points.size();

            /// 初始化ikdtree。
            if(iKdTree.Root_Node == nullptr) {
                if(G_num_dsampled_features > 5) {
                    iKdTree.set_downsample_param(fastlio::options::opt_vf_size_map);
                    features_dsampled_in_world_->resize(G_num_dsampled_features);
                    for(int i = 0; i < G_num_dsampled_features; i++)
                    {
                        PointBodyToWorld(&(features_dsampled_in_body_->points[i]), 
                            &(features_dsampled_in_world_->points[i]), current_state);
                    }
                    iKdTree.Build(features_dsampled_in_world_->points);
                }
                continue;
            }
            int num_ikdtree_points = iKdTree.validnum();
            kdtree_size_st = iKdTree.size();
            
            // std::cout << "[ mapping ]: In num: " << laser_features_deskewed_->points.size() << " downsamp " << G_num_dsampled_features
            //     << " Map num: " << num_ikdtree_points << "effect num:" << G_num_matched_features << std::endl;

            /// ICP and iterated Kalman filter update
            if (G_num_dsampled_features < 5)
            {
                std::cout << "Too less features, skip this scan!" << std::endl;
                continue;
            }
            
            features_norms_info->resize(G_num_dsampled_features);
            features_dsampled_in_world_->resize(G_num_dsampled_features);

            V3D ext_euler = SO3ToEuler(current_state.offset_R_L_I);
            fout_pre << std::setw(20) << measures_.lidar_beg_time - G_first_lidar_time << " " << current_euler.transpose()
                << " " << current_state.pos.transpose() << " " << ext_euler.transpose() 
                << " " << current_state.offset_T_L_I.transpose() << " " << current_state.vel.transpose() \
                <<  " " << current_state.bg.transpose() << " " << current_state.ba.transpose() << " " << current_state.grav << std::endl;

            /// 可视化ikdtree地图
            if(visulize_ikdtree_points)
            {
                PointVector().swap(iKdTree.PCL_Storage);
                iKdTree.flatten(iKdTree.Root_Node, iKdTree.PCL_Storage, ikdtreeNS::NOT_RECORD);
                points_from_ikdtree_->clear();
                points_from_ikdtree_->points = iKdTree.PCL_Storage;
                ikdtree_points_updated = true;
                std::cout << "get ikdtree points num: " << points_from_ikdtree_->size() << std::endl;
            }

            /// 在执行"h_share_model"步骤之前，先把变量resize完毕
            features_nearest_neighbors.resize(G_num_dsampled_features);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            /// ieskf算法
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            ieskf_estimator_.update_iterated_dyn_share_modified(fastlio::options::opt_laser_point_cov, solve_H_time);
            current_state = ieskf_estimator_.get_x();
            current_euler = SO3ToEuler(current_state.rot);
            current_lidar_posi = current_state.pos + current_state.rot * current_state.offset_T_L_I;
            double t_update_end = omp_get_wtime();

            /// 更新ikdtree。
            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            UpdateIkdtreeMap();
            t5 = omp_get_wtime();

            /*** Debug variables ***/
            if (fastlio::options::opt_enable_runtime_logging)
            {
                frame_num ++;
                kdtree_size_end = iKdTree.size();
                avg_time_consu = avg_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                avg_time_icp = avg_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                avg_time_match = avg_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                avg_time_incre = avg_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                avg_time_solve = avg_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                avg_time_const_H_time = avg_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[logging_count] = measures_.lidar_beg_time;
                s_plot[logging_count] = t5 - t0;
                s_plot2[logging_count] = laser_features_deskewed_->points.size();
                s_plot3[logging_count] = kdtree_incremental_time;
                s_plot4[logging_count] = kdtree_search_time;
                s_plot5[logging_count] = num_deleted_points;
                s_plot6[logging_count] = kdtree_delete_time;
                s_plot7[logging_count] = kdtree_size_st;
                s_plot8[logging_count] = kdtree_size_end;
                s_plot9[logging_count] = avg_time_consu;
                s_plot10[logging_count] = num_added_points;
                logging_count ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",
                    t1-t0, avg_time_match, avg_time_solve, t3-t1, t5-t3, avg_time_consu, avg_time_icp, avg_time_const_H_time);
                ext_euler = SO3ToEuler(current_state.offset_R_L_I);
                fout_out << std::setw(20) << measures_.lidar_beg_time - G_first_lidar_time << " " << current_euler.transpose() 
                    << " " << current_state.pos.transpose() << " "<< ext_euler.transpose() << " " << current_state.offset_T_L_I.transpose()
                    << " " << current_state.vel.transpose() << " "<< current_state.bg.transpose() << " " <<current_state.ba.transpose()
                    << " " << current_state.grav << " " << laser_features_deskewed_->points.size() << std::endl;
                DumpLioStateToLog(fp_lio_state, current_state, measures_);
            }

            /******* Publish odometry *******/
            PublishLidarOdometry(pubOdomAftMapped);

            /******* Publish points *******/
            if (enable_pub_path) { PublishLioPath(pubLioPath); }
            if (enable_pub_scan || fastlio::options::opt_enable_save_pcds) 
            { PublishKfPointsInWorld(pubLaserCloudRegistered); }
            // if (enable_pub_scan && enable_pub_body_scan) PublishKfPointsInBody(pubLaserCloudRegisInBody);
            PublishPointsMatchedSurfs(pubLaserCloudMatchedSurfs);
            PublishIkdtreeMap(pubIkdtreeMap);

        }

        ros_ok = ros::ok();
        rate.sleep();
    }

    /** ##保存点云地图到pcd##
     * save map:
     * 1. make sure you have enough memories
     * 2. pcd save will largely influence the real-time performences
    */
    if (pcl_wait_save->size() > 0 && fastlio::options::opt_enable_save_pcds)
    {
        std::string file_name = std::string("scans.pcd");
        if (pcd_file_id > 0) {
            file_name = std::string("scans_") + std::to_string(pcd_file_id) + std::string(".pcd");
        }
        std::string pcd_file_path(std::string(fastlio::options::opt_root_directory + "PCD/") + file_name);
        std::cout << "going to save pcd file: " << pcd_file_path << std::endl;
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(pcd_file_path, *pcl_wait_save);
    }

    if (fout_out) { fout_out.close(); }
    if (fout_pre) { fout_pre.close(); }

    if (fastlio::options::opt_enable_runtime_logging)
    {
        std::vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        std::string log_dir = fastlio::options::opt_root_directory + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<logging_count; i++){
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n", 
                T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],
                int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;

    /// TODO: rename to run_fastlio_main.cpp
}
