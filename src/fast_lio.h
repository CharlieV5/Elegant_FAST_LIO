/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#ifndef FASTLIO_FAST_LIO_H_
#define FASTLIO_FAST_LIO_H_

#include <mutex>
#include <cmath>
#include <thread>
#include <fstream>
#include <csignal>
#include <condition_variable>
#include <functional> // std::bind, std::function

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


namespace fastlio {
} // namespace fastlio

struct RunningStatus {
    // 1. 状态量
    // 2. 各种耗时
};


/// @brief 算法层顶层类，考虑是否多线程安全
class FastLio {
   public: // ========================== 公有函数 ========================== //
    FastLio();
    ~FastLio();

    void Init();

    // 暂时无必要实现
    // void Reset();
    // void SetExtrinsic(const V3D& lidar2imu_T, const M3D& lidar2imu_R);
    // void EnableRuntimeLogging(const bool& enable);

    // 运行一次LIO的入口
    bool ProcessMeasurements(MeasureGroup& measures_);

    // 获取LIO状态(位姿)
    state_ikfom GetCurrentState() { return current_state; }
    // IeskfomType::cov GetCurrentP();
    // void GetPosesList();

    // 获取点云
    PointCloudXYZI::Ptr GetKfPointsDeskewed() { return laser_features_deskewed_; }
    PointCloudXYZI::Ptr GetKfPointsDsampled() { return features_dsampled_in_body_; }
    PointCloudXYZI::Ptr GetKfPointsMatched() { return laser_features_matched_; }
    size_t GetNumMatchedFeatures() { if (laser_features_matched_) { return laser_features_matched_->size(); } return 0; }

    // 获取ikdtree(local map)点云
    bool IsIkdtreePointsUpdated() { bool is_updated = ikdtree_points_updated; ikdtree_points_updated = false; return is_updated; }
    void EnableIkdtreePoints(const bool& enable) { visulize_ikdtree_points = enable; }
    PointCloudXYZI::Ptr GetIkdtreePoints() { return points_from_ikdtree_; }

    // 获取运行状态信息
    // RunningStatus GetRunningStatus();

    // 后处理
    void SavePcdsIfNecessary(const bool final_saving = false);
    void SaveRuntimeLogIfNecessary();


   public: // ========================== 公有函数 ========================== //
    //

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


   public: // ========================== 公有成员 ========================== //

    // 保存地图
    PointCloudXYZI::Ptr pcl_wait_save = nullptr; //(new PointCloudXYZI());
    int    pcd_file_id = 0;

    // Time Log Variables
    int    frame_num = 0;
    double avg_time_consu = 0, avg_time_icp = 0, avg_time_match = 0;
    double avg_time_incre = 0, avg_time_solve = 0, avg_time_const_H_time = 0;

    double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
    double T1[kMaxLogN], s_plot[kMaxLogN], s_plot2[kMaxLogN], s_plot3[kMaxLogN], s_plot4[kMaxLogN], s_plot5[kMaxLogN];
    double s_plot6[kMaxLogN], s_plot7[kMaxLogN], s_plot8[kMaxLogN], s_plot9[kMaxLogN], s_plot10[kMaxLogN];
    double s_plot11[kMaxLogN]; // 预处理耗时统计(位于ros层)
    double match_time = 0, solve_time = 0, solve_const_H_time = 0;
    int    kdtree_size_st = 0, kdtree_size_end = 0, num_added_points = 0, num_deleted_points = 0; /*kdtree_delete_counter*/

    // 逻辑：类内变量
    bool   flag_first_scan = true, flag_SCAN_inited = false;
    double G_first_lidar_time = 0.0, G_lidar_end_time = 0, G_total_distance = 0;
    int    received_scan_count = 0, logging_count = 0;
    int    iterCount = 0, laserCloudValidNum = 0;
    double residual_mean_last = 0.05, total_residual = 0.0;

    // 预处理
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    // 特征点云
    int    G_num_dsampled_features = 0; // 预处理后的scan降采样后才能给到算法用
    int    G_num_matched_features = 0;  // 一个观测单元中，所有match成功的feature的数量 
    PointCloudXYZI::Ptr laser_features_deskewed_ = nullptr; //(new PointCloudXYZI());   // 预处理(会跳点!)后经运动补偿且转移到IMU系的所有点
    PointCloudXYZI::Ptr features_dsampled_in_body_ = nullptr; //(new PointCloudXYZI()); // 体素降采样后的点，也是算法真正用的点
    PointCloudXYZI::Ptr features_dsampled_in_world_ = nullptr; //(new PointCloudXYZI());// 和body系下的点一一对应，会被酌情添加到ikdtree地图中

    // 是否关联到了有效平面(也即match)
    std::vector<PointVector>  features_nearest_neighbors;    // 保存每个feature点的NN近邻点
    bool features_matched_info[100000] = {0};           // 各个feature是否成功match（在kdtree中的近邻形成了有效平面）的标签
    float features_residuals_info[100000] = {0.0};      // 所有match点的点面距离残差
    PointCloudXYZI::Ptr features_norms_info = nullptr; //(new PointCloudXYZI(100000, 1));
    PointCloudXYZI::Ptr laser_features_matched_ = nullptr; //(new PointCloudXYZI(100000, 1)); // 成功match的特征点
    PointCloudXYZI::Ptr matched_norms_ = nullptr; //(new PointCloudXYZI(100000, 1));
    PointCloudXYZI::Ptr features_arrayed_ = nullptr; //; // unused, 存储从ikdtree中remove掉的点，大概是用于debug

    // ikdtree相关
    bool visulize_ikdtree_points = false;
    bool ikdtree_points_updated = false;
    PointCloudXYZI::Ptr points_from_ikdtree_ = nullptr; //(new PointCloudXYZI()); // ikdtree点云地图可视化
    std::vector<ikdtreeNS::BoxPointType> boxes_need_remove;     // 每次trim localmap时，需要被裁剪掉的区域

    // 未启用
    V3F XAxisPoint_body = V3F(LIDAR_SP_LEN, 0.0, 0.0);
    V3F XAxisPoint_world = V3F(LIDAR_SP_LEN, 0.0, 0.0);

    // 算法设施
    ikdtreeNS::KD_TREE<PointType> iKdTree;
    std::shared_ptr<ImuProcess> imu_processor_ = nullptr; //(new ImuProcess());

    // EKF相关
    IeskfomType ieskf_estimator_;
    state_ikfom current_state;
    vect3 current_lidar_posi;
    QuatD current_quat = QuatD::Identity();
    V3D current_euler;

    // 其它
    FILE *fp_lio_state;
    std::ofstream fout_pre, fout_out, fout_dbg;


   private: // ========================== 私有函数 ========================== //
    // 


   private: // ========================== 私有成员 ========================== //
    //

};




#endif // FASTLIO_FAST_LIO_H_