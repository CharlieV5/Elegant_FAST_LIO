/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#ifndef FASTLIO_MSG_CONVERSION_H_
#define FASTLIO_MSG_CONVERSION_H_

#include <string>
#include <vector>
#include <memory>

#include <boost/shared_ptr.hpp>
#include <boost/make_unique.hpp>

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

#include "math/so3_math.h"
#include "common/sensor_type.h"

/// @brief ROS消息类型转内置类型
inline fastlio::ImuData::Ptr ToImuData(const sensor_msgs::Imu &msg) 
{
    fastlio::ImuData::Ptr imu_data(new fastlio::ImuData());
    imu_data->timestamp = msg.header.stamp.toSec();
    imu_data->orientation.w() = msg.orientation.w;
    imu_data->orientation.x() = msg.orientation.x;
    imu_data->orientation.y() = msg.orientation.y;
    imu_data->orientation.z() = msg.orientation.z;
    // todo: covariance (暂不管,算法内部没用这些量)
    imu_data->angular_velocity.x() = msg.angular_velocity.x;
    imu_data->angular_velocity.y() = msg.angular_velocity.y;
    imu_data->angular_velocity.z() = msg.angular_velocity.z;
    // todo: covariance (暂不管,算法内部没用这些量)
    imu_data->linear_acceleration.x() = msg.linear_acceleration.x;
    imu_data->linear_acceleration.y() = msg.linear_acceleration.y;
    imu_data->linear_acceleration.z() = msg.linear_acceleration.z;
    // todo: covariance (暂不管,算法内部没用这些量)

    return imu_data;
}

/// @brief ROS消息类型转内置类型
inline fastlio::ImuData::Ptr ToImuData(const sensor_msgs::Imu::ConstPtr &msg) 
{
    return ToImuData(*msg);
}

/// @brief Livox激光雷达消息类型(Livox点云)转内置类型
/// @brief 普通激光雷达消息类型(ROS点云)转内置类型



#endif // FASTLIO_MSG_CONVERSION_H_