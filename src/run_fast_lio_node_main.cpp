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

#include "common/options.h"
#include "common/utils.h"
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

std::shared_ptr<FastLio> fast_lio_;

// *************** ROS callback ***************

void HandleCommonPointCloudMsg(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mapping_mtx_.lock();

    /// 计数计时
    fast_lio_->received_scan_count ++;
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
    if (fast_lio_->received_scan_count < kMaxLogN) 
    {
        fast_lio_->s_plot11[fast_lio_->received_scan_count] = 
            omp_get_wtime() - preprocess_start_time;
    }
    mapping_mtx_.unlock();
    condition_var_.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void HandleLivoxPointCloudMsg(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    mapping_mtx_.lock();
    double preprocess_start_time = omp_get_wtime();
    fast_lio_->received_scan_count ++;
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
    if (fast_lio_->received_scan_count < kMaxLogN) 
    {
        fast_lio_->s_plot11[fast_lio_->received_scan_count] = 
            omp_get_wtime() - preprocess_start_time;
    }
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
            fast_lio_->G_lidar_end_time = measurements.lidar_beg_time + mean_per_scan_time;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (measurements.lidar->points.back().curvature / double(1000) < 0.5 * mean_per_scan_time) {
            ROS_WARN("Lidar scan duration is shorter than expected.\n");
            fast_lio_->G_lidar_end_time = measurements.lidar_beg_time + mean_per_scan_time;
        }
        else {
            num_scans_++;
            fast_lio_->G_lidar_end_time = measurements.lidar_beg_time + measurements.lidar->points.back().curvature / double(1000);
            mean_per_scan_time += (measurements.lidar->points.back().curvature / double(1000) - mean_per_scan_time) / num_scans_;
        }

        measurements.lidar_end_time = fast_lio_->G_lidar_end_time;
        flag_lidar_pushed = true;
    }

    /// 要求imu时段必须完全覆盖lidar scan时段
    if (last_timestamp_Imu < fast_lio_->G_lidar_end_time) {
        return false;
    }

    /// push imu data, and pop from imu buffer
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    // measurements.imu.clear();
    measurements.imu_queue.clear();
    while ((!imu_buffer.empty()) && (imu_time < fast_lio_->G_lidar_end_time)) {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > fast_lio_->G_lidar_end_time) { break; }
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
    auto laser_features_deskewed_ = fast_lio_->GetKfPointsDeskewed();
    auto features_dsampled_in_body_ = fast_lio_->GetKfPointsDsampled();
    auto current_state = fast_lio_->GetCurrentState();
    auto G_lidar_end_time = fast_lio_->G_lidar_end_time;
    if (!laser_features_deskewed_ || !features_dsampled_in_body_) { return; }

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
    fast_lio_->SavePcdsIfNecessary();
}

void PublishKfPointsInBody(const ros::Publisher & pubLaserCloudRegisInBody)
{
    auto laser_features_deskewed_ = fast_lio_->GetKfPointsDeskewed();
    auto features_dsampled_in_body_ = fast_lio_->GetKfPointsDsampled();
    auto current_state = fast_lio_->GetCurrentState();
    auto G_lidar_end_time = fast_lio_->G_lidar_end_time;
    if (!laser_features_deskewed_ || !features_dsampled_in_body_) { return; }

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
    auto laser_features_matched_ = fast_lio_->GetKfPointsMatched();
    auto current_state = fast_lio_->GetCurrentState();
    auto G_num_matched_features = fast_lio_->G_num_matched_features;
    auto G_lidar_end_time = fast_lio_->G_lidar_end_time;
    if (!laser_features_matched_) { return; }

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
        // visulize_ikdtree_points = true;
        fast_lio_->EnableIkdtreePoints(true);
        if (!fast_lio_->IsIkdtreePointsUpdated()) 
        { 
            std::cout << "ikdtree points not updated, failed to publish ikdtree map ..." << std::endl;
            return; 
        }
        auto points_from_ikdtree_ = fast_lio_->GetIkdtreePoints();
        auto G_lidar_end_time = fast_lio_->G_lidar_end_time;
        sensor_msgs::PointCloud2 laserCloudMap;
        pcl::toROSMsg(*points_from_ikdtree_, laserCloudMap);
        laserCloudMap.header.stamp = ros::Time().fromSec(G_lidar_end_time);
        laserCloudMap.header.frame_id = kWorldFrameId;
        pubIkdtreeMap.publish(laserCloudMap);
        // ikdtree_points_updated = false;
    } 
    else 
    {
        // visulize_ikdtree_points = false;
        fast_lio_->EnableIkdtreePoints(false);
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
    auto current_state = fast_lio_->GetCurrentState();
    auto G_lidar_end_time = fast_lio_->G_lidar_end_time;
    auto P = fast_lio_->ieskf_estimator_.get_P();

    odomAftMapped.header.frame_id = kWorldFrameId;
    odomAftMapped.child_frame_id = kBodyFrameId;
    odomAftMapped.header.stamp = ros::Time().fromSec(G_lidar_end_time);// ros::Time().fromSec(G_lidar_end_time);
    SetRosPose(odomAftMapped.pose, current_state);
    pubOdomAftMapped.publish(odomAftMapped);

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
    auto current_state = fast_lio_->GetCurrentState();
    auto G_lidar_end_time = fast_lio_->G_lidar_end_time;

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

    if (fastlio::options::opt_save_pcd_every_n < 0)
    {
        fastlio::options::opt_save_pcd_every_n = 100;
    }

    fastlio::options::opt_root_directory = ROOT_DIR;

    // core test
    fast_lio_.reset(new FastLio);
    fast_lio_->Init();

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
            /******* FastLio算法层 *******/
            fast_lio_->ProcessMeasurements(measures_);

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

    fast_lio_->SavePcdsIfNecessary(true);
    fast_lio_->SaveRuntimeLogIfNecessary();

    return 0;
}
