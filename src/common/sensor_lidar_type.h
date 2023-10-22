/**
 * Developer: WANG_Guanhua on 20231014, Saturday
*/

#ifndef FASTLIO_COMMON_SENSOR_LIDAR_TYPE_H_
#define FASTLIO_COMMON_SENSOR_LIDAR_TYPE_H_

#include <string>
#include <vector>
#include <memory>

#include <boost/shared_ptr.hpp>
#include <boost/make_unique.hpp>

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace fastlio {
} // namespace fastlio


// 适配于不同型号LiDAR的点云数据类型，基于PCL实现，常用于ROS层（ros格式点云转特定格式点云）
/** NOTE: PointXYZINormal中，curvature代表点的时间戳，单位为毫秒[ms]!
 *  NOTE: 全部成员：{x, y, z, normal_x, normal_x, normal_x, curvature, intensity}。
*/
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum LIDAR_TYPE{AVIA = 1, VELO16, OUST64, HESAI128};
enum FEATURE_TYPE{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum SURROUND_TYPE{Prev, Next};
enum E_JUMP_TYPE{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

// 输出雷达类型的字符串
inline std::string ToLidarString(const LIDAR_TYPE& type) {
  if (type == LIDAR_TYPE::AVIA) return "AVIA";
  if (type == LIDAR_TYPE::VELO16) return "VELO16";
  if (type == LIDAR_TYPE::OUST64) return "OUST64";
  if (type == LIDAR_TYPE::HESAI128) return "HESAI128";
  return "unknown lidar type!";
}

inline std::string ToLidarString(const int& type) {
  if (type == LIDAR_TYPE::AVIA) return "AVIA";
  if (type == LIDAR_TYPE::VELO16) return "VELO16";
  if (type == LIDAR_TYPE::OUST64) return "OUST64";
  if (type == LIDAR_TYPE::HESAI128) return "HESAI128";
  return "unknown lidar type!";
}

/// @brief 具体指的是(LiDAR)预处理的类型？
struct OrgType
{
  double range;
  double dista; 
  double angle[2];
  double intersect;
  E_JUMP_TYPE edj[2];
  FEATURE_TYPE ftype;
  OrgType()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

namespace velodyne_pcl {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pcl::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace ouster_pcl {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_pcl::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

namespace lidar128_pcl {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint16_t ring;
      uint16_t column;
      float distance;
      float time;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar128_pcl::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (uint16_t, column, column)
    (float, distance, distance)
    (float, time, time)
)



#endif // FASTLIO_COMMON_SENSOR_LIDAR_TYPE_H_