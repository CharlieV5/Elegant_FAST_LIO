/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#include "fast_lio.h"

#include <chrono>

namespace fastlio {
} // namespace fastlio


FastLio::FastLio() 
{
    /// 点云指针初始化
    pcl_wait_save.reset(new PointCloudXYZI());
    laser_features_deskewed_.reset(new PointCloudXYZI());
    features_dsampled_in_body_.reset(new PointCloudXYZI());
    features_dsampled_in_world_.reset(new PointCloudXYZI());

    features_norms_info.reset(new PointCloudXYZI(100000, 1));
    laser_features_matched_.reset(new PointCloudXYZI(100000, 1));
    matched_norms_.reset(new PointCloudXYZI(100000, 1));
    features_arrayed_.reset(new PointCloudXYZI());

    map_points_matched_.reset(new PointCloudXYZI());
    points_from_ikdtree_.reset(new PointCloudXYZI());

    // 核心成员初始化
    imu_processor_.reset(new ImuProcess());

}

FastLio::~FastLio() 
{
    //

}

void FastLio::Init()
{
    const double kVFSizeSurf = fastlio::options::opt_vf_size_surf;
    downSizeFilterSurf.setDownsampleAllData(true);
    downSizeFilterSurf.setLeafSize(kVFSizeSurf, kVFSizeSurf, kVFSizeSurf);

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
    // auto h_model_func_ = std::bind(&FastLio::h_share_model, this, std::placeholders::_1, std::placeholders::_2);
    auto h_model_func_ = [this](state_ikfom &ikfom_state, esekfom::dyn_share_datastruct<double> &ekfom_data) { 
        this->h_share_model(ikfom_state, ekfom_data); };
    ieskf_estimator_.init_dyn_share(get_f, df_dx, df_dw, h_model_func_,
        fastlio::options::opt_ekf_max_iterations, epsi);

    // debug recorder
    std::string pos_log_dir = fastlio::options::opt_root_directory + "/Log/pos_log.txt";
    fp_lio_state = fopen(pos_log_dir.c_str(),"w");
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),std::ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),std::ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),std::ios::out);
    if (fout_pre && fout_out) {
        std::cout << " [ root directory ] " << fastlio::options::opt_root_directory << " files opened." << std::endl;
    } else {
        std::cout << " [ root directory ] " << fastlio::options::opt_root_directory << " doesn't exist." << std::endl;
    }


    return;
}

// 运行一次LIO的入口
bool FastLio::ProcessMeasurements(MeasureGroup& measures_)
{
    using StdSteadyClock = std::chrono::steady_clock;
    auto proc_start_time_point = StdSteadyClock::now();
 
    /// 无条件跳过第一帧 (IMUProc初始化也不需要这个,有点奇怪)
    if (flag_first_scan)
    {
        G_first_lidar_time = measures_.lidar_beg_time;
        imu_processor_->first_lidar_time = G_first_lidar_time;
        flag_first_scan = false;
        // continue;
        return false;
    }

    /// ***********************************************************************************

    running_status_ = RunningStatus();
    double t0, t1, t2, t3, t4, t5;
    // double match_start(0), solve_start(0), svd_time(0);
    t0 = omp_get_wtime();

    /// 滤波器状态预测 + 运动补偿(同时也把点云从lidar坐标系下转移到IMU坐标系下)
    /*注意，这里意味着IMUProc即使未初始化成功（意味着无点云运动补偿），也会继续往下进行 */
    /*讨论：看了这个函数体，不存在IMU(预)积分更新滤波器预测的情形，所以没平移预测啊 */
    const bool inited = imu_processor_->Process(measures_, ieskf_estimator_, laser_features_deskewed_ /*output*/);
    current_state = ieskf_estimator_.get_x(); /*状态量更新为预测值*/
    current_lidar_posi = current_state.pos + current_state.rot * current_state.offset_T_L_I;

    if (laser_features_deskewed_->empty() || (laser_features_deskewed_ == NULL))
    {
        std::cout << "No point, skip this scan!" << std::endl;
        // continue;
        return false;
    }

    /// 这是个偷懒的的行为(并不代表事实上是否真的初始化了)
    flag_SCAN_inited = (measures_.lidar_beg_time - G_first_lidar_time) 
        < fastlio::options::opt_ekf_wait_lidar_secs ? false : true;

    /// scan降采样
    G_num_input_features = laser_features_deskewed_->points.size();
    auto points_to_dsample = laser_features_deskewed_;
    if (fastlio::options::opt_colorize_output_features) {
        /* NOTE: 按照features着色，只允许在降采样及以后的点云中；deskewed点云要求保留原始信息 */
        /* 另外，pcl的体素滤波貌似会把normal信息重写，因此，需要把着色信息从normal转移到intensity中 */
        points_to_dsample.reset(new PointCloudXYZI(*laser_features_deskewed_));
        for (auto& pt : points_to_dsample->points) {
            pt.intensity = pt.normal_z;
        }
    }
    downSizeFilterSurf.setInputCloud(points_to_dsample);
    downSizeFilterSurf.filter(*features_dsampled_in_body_);
    G_num_dsampled_features = features_dsampled_in_body_->points.size();
    if (fastlio::options::opt_colorize_output_features) {
        // ============== debug log ==============
        std::set<int> values;
        for (auto pt : laser_features_deskewed_->points) {
            int value = int(pt.normal_z);
            if (values.find(value) == values.end()) {
                values.insert(value);
            }
        }
        std::cout << "input pts intensities: ";
        for (auto value : values) {
            std::cout << value << ", ";
        }
        std::cout << std::endl;
        values.clear();
        for (auto pt : features_dsampled_in_body_->points) {
            int value = int(pt.intensity);
            if (values.find(value) == values.end()) {
                values.insert(value);
            }
        }
        std::cout << "dsampled pts intensities: ";
        for (auto value : values) {
            std::cout << value << ", ";
        }
        std::cout << std::endl;
    }

    /// 动态裁剪localMap
    TrimLocalMapAdaptively();
    t1 = omp_get_wtime(); /*三合一耗时统计*/

    /// ***********************************************************************************

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
        // continue;
        return false;
    }
    int num_ikdtree_points = iKdTree.validnum();
    kdtree_size_st = iKdTree.size();

    // std::cout << "[ mapping ]: In num: " << G_num_input_features << " downsamp " << G_num_dsampled_features 
    //     << " Map valid num: " << num_ikdtree_points << " Map num:" << kdtree_size_st << std::endl;

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

    /// ICP and iterated Kalman filter update
    if (G_num_dsampled_features < 5)
    {
        std::cout << "Too less features, skip this scan!" << std::endl;
        // continue;
        return false;
    }
    
    /// 在执行"h_share_model"步骤之前，先把变量resize完毕
    features_dsampled_in_world_->resize(G_num_dsampled_features);
    features_nearest_neighbors.resize(G_num_dsampled_features);
    features_norms_info->resize(G_num_dsampled_features);

    V3D ext_euler = SO3ToEuler(current_state.offset_R_L_I);
    fout_pre << std::setw(20) << measures_.lidar_beg_time - G_first_lidar_time << " " << current_euler.transpose()
        << " " << current_state.pos.transpose() << " " << ext_euler.transpose() 
        << " " << current_state.offset_T_L_I.transpose() << " " << current_state.vel.transpose() \
        <<  " " << current_state.bg.transpose() << " " << current_state.ba.transpose() << " " << current_state.grav << std::endl;

    // reset time-related variables.
    G_ieskf_ites_ = 0;              // 表征ieskf迭代中：迭代次数
    G_match_time = 0;               // 表征ieskf迭代中：为feature寻找关联的耗时
    G_kdtree_search_time = 0.0;     // 表征ieskf迭代中：未启用
    G_solve_time = 0;               // 表征ieskf迭代中：求解IESKF的耗时
    G_solve_const_H_time = 0;       // 表征ieskf迭代中：未启用

    int  rematch_num = 0;           // unused.
    bool nearest_search_en = true;  // unused.
    t2 = omp_get_wtime();

    /// ***********************************************************************************

    /// ieskf算法
    /*** iterated state estimation ***/
    double t_update_start = omp_get_wtime();
    double solve_H_time = 0;    /*计算H矩阵耗时？*/
    ieskf_estimator_.update_iterated_dyn_share_modified(fastlio::options::opt_laser_point_cov, solve_H_time);
    current_state = ieskf_estimator_.get_x();
    current_euler = SO3ToEuler(current_state.rot);
    current_lidar_posi = current_state.pos + current_state.rot * current_state.offset_T_L_I;
    current_quat.x() = current_state.rot.coeffs()[0];
    current_quat.y() = current_state.rot.coeffs()[1];
    current_quat.z() = current_state.rot.coeffs()[2];
    current_quat.w() = current_state.rot.coeffs()[3];
    double t_update_end = omp_get_wtime();
    t3 = omp_get_wtime();

    /// ***********************************************************************************

    /// 更新ikdtree。
    /*** add the feature points to map kdtree ***/
    UpdateIkdtreeMap();
    t5 = omp_get_wtime();

    /// ***********************************************************************************

    /// debug info
    double proc_duration = std::chrono::duration_cast<std::chrono::duration<double>>(
            StdSteadyClock::now() - proc_start_time_point).count();
    static size_t process_cnts = 0;
    process_cnts++;
    std::cout << "[ FastLIO ] input " << G_num_input_features << ", dsampled " << G_num_dsampled_features 
        << ", matched " << G_num_matched_features << ", added " << G_num_added_points << ", seq=" << process_cnts << std::endl;
    printf("[ FastLIO ] lidarPreproc %0.4f, 3on1 %0.4f, visIkdtree %0.4f, IESKF %0.4f, updateIkdtree %0.4f, Total %0.4f, stdTotal %0.4f \n", 
        G_lidar_preproc_time, t1-t0, t2-t1, t3-t2, t5-t3, t5-t0, proc_duration);
    printf("[  IESKF  ] Iterations %d, matchTime %0.4f, solveTime %0.4f \n", G_ieskf_ites_, G_match_time, G_solve_time);

    /*** Debug variables ***/
    if (fastlio::options::opt_enable_runtime_logging)
    {
        frame_num ++;
        kdtree_size_end = iKdTree.size();
        avg_time_consu = avg_time_consu * (frame_num - 1)/frame_num + (t5 - t0) / frame_num; /*增量式求均值*/
        avg_time_icp = avg_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
        avg_time_match = avg_time_match * (frame_num - 1)/frame_num + (G_match_time)/frame_num;
        avg_time_incre = avg_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
        avg_time_solve = avg_time_solve * (frame_num - 1)/frame_num + (G_solve_time + solve_H_time)/frame_num;
        avg_time_const_H_time = avg_time_const_H_time * (frame_num - 1)/frame_num + G_solve_time / frame_num;
        T1[logging_count] = measures_.lidar_beg_time;                       // 时间戳
        s_plot01[logging_count] = t5 - t0;                                  // 总耗时
        s_plot02[logging_count] = G_num_input_features;                     // 输入feature数量
        s_plot03[logging_count] = kdtree_incremental_time;                  // ikdtree更新
        s_plot04[logging_count] = G_kdtree_search_time;                     // ikdtree近邻搜索【无效】
        s_plot05[logging_count] = num_deleted_points;                       // ikdtree删除
        s_plot06[logging_count] = kdtree_delete_time;                       // ikdtree删除耗时
        s_plot07[logging_count] = kdtree_size_st;                           // ikdtree起始size
        s_plot08[logging_count] = kdtree_size_end;                          // ikdtree更新后size
        s_plot09[logging_count] = avg_time_consu;                           // 近期平均耗时
        s_plot10[logging_count] = G_num_added_points;                       // ikdtree新增点数
        s_plot11[logging_count] = G_lidar_preproc_time;                     // ROS层RawScan预处理耗时
        logging_count ++;
        printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f \
                map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",
            t1-t0, avg_time_match, avg_time_solve, t3-t1, t5-t3, avg_time_consu, avg_time_icp, avg_time_const_H_time);
        ext_euler = SO3ToEuler(current_state.offset_R_L_I);
        // 写入文档
        fout_out << std::setw(20) << measures_.lidar_beg_time - G_first_lidar_time << " " << current_euler.transpose() 
            << " " << current_state.pos.transpose() << " "<< ext_euler.transpose() << " " << current_state.offset_T_L_I.transpose()
            << " " << current_state.vel.transpose() << " "<< current_state.bg.transpose() << " " <<current_state.ba.transpose()
            << " " << current_state.grav << " " << G_num_input_features << std::endl;
        DumpLioStateToLog(fp_lio_state, current_state, measures_);
    }

    return false;
}

void FastLio::SavePcdsIfNecessary(const bool final_saving)
{
    /** ****** save map ******
     * save map:
     * 1. make sure you have enough memories
     * 2. pcd save will largely influence the real-time performences
    */

    if (!fastlio::options::opt_enable_save_pcds) { return; }

    // on-running save.
    if (!final_saving) 
    {
        const int size = laser_features_deskewed_->points.size();
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

        return;
    }

    // final save.
    if (pcl_wait_save->size() > 0)
    {
        std::string file_name = std::string("scans_all.pcd");
        if (pcd_file_id > 0) {
            file_name = std::string("scans_") + std::to_string(pcd_file_id) + std::string(".pcd");
        }
        std::string pcd_file_path(std::string(fastlio::options::opt_root_directory + "PCD/") + file_name);
        std::cout << "going to save pcd file: " << pcd_file_path << std::endl;
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(pcd_file_path, *pcl_wait_save);
    }
}

void FastLio::SaveRuntimeLogIfNecessary()
{
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
                T1[i],s_plot01[i],int(s_plot02[i]),s_plot03[i],s_plot04[i],
                int(s_plot05[i]),s_plot06[i],int(s_plot07[i]),int(s_plot08[i]), 
                int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot09[i]);
            s_vec2.push_back(s_plot03[i] + s_plot06[i]);
            s_vec3.push_back(s_plot04[i]);
            s_vec5.push_back(s_plot01[i]);
        }
        fclose(fp2);
    }
}






