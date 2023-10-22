/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#include "fast_lio.h"

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
        std::cout << "~~~~" << fastlio::options::opt_root_directory << " file opened" << std::endl;
    } else {
        std::cout << "~~~~" << fastlio::options::opt_root_directory << " doesn't exist" << std::endl;
    }


    return;
}

// 运行一次LIO的入口
bool FastLio::ProcessMeasurements(MeasureGroup& measures_)
{
    //
    /// 无条件跳过第一帧？(IMUProc初始化也不需要这个,有点奇怪)
    if (flag_first_scan)
    {
        G_first_lidar_time = measures_.lidar_beg_time;
        imu_processor_->first_lidar_time = G_first_lidar_time;
        flag_first_scan = false;
        // continue;
        return false;
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
        // continue;
        return false;
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
        // continue;
        return false;
    }
    int num_ikdtree_points = iKdTree.validnum();
    kdtree_size_st = iKdTree.size();
    
    // std::cout << "[ mapping ]: In num: " << laser_features_deskewed_->points.size() << " downsamp " << G_num_dsampled_features
    //     << " Map num: " << num_ikdtree_points << "effect num:" << G_num_matched_features << std::endl;

    /// ICP and iterated Kalman filter update
    if (G_num_dsampled_features < 5)
    {
        std::cout << "Too less features, skip this scan!" << std::endl;
        // continue;
        return false;
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
    current_quat.x() = current_state.rot.coeffs()[0];
    current_quat.y() = current_state.rot.coeffs()[1];
    current_quat.z() = current_state.rot.coeffs()[2];
    current_quat.w() = current_state.rot.coeffs()[3];
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

    return false;
}

void FastLio::SavePcdsIfNecessary(const bool final_saving)
{
    /** ****** save map ******
     * save map:
     * 1. make sure you have enough memories
     * 2. pcd save will largely influence the real-time performences
    */

    // on-running save.
    if (!final_saving) 
    {
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
        return;
    }

    // final save.
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
}






