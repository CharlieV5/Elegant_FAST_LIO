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

enum TIME_UNIT { SEC = 0, MS = 1, US = 2, NS = 3 };
enum LIDAR_TYPE { AVIA = 1, VELO16, OUST64, HESAI128 };
enum SURROUND_TYPE { Prev = 0, Next };

enum FEATURE_TYPE {
  Nor = 0,    // 啥也不是？
  Poss_Plane, // Possible surf，多见于group的两端点
  Real_Plane, // Real surf，多见于group中的点
  Edge_Jump,  // corner，盲猜应该是和Neighbor距离较大的点，也即距离跳跃的edge点
  Edge_Plane, // corner，尤指两个plane相交形成的edge点
  Wire,       // 电线或类似形状物体上的点
  ZeroPoint   // ？
};

enum E_JUMP_TYPE {  // Edge_Jump点的细分类型（盲猜Nr指的是Neighbor Relation？）
  Nr_nor = 0, // 啥也不是？
  Nr_zero,    // 邻点在延长线上
  Nr_180,     // 邻点在原点到当前点(OA)的连线上
  Nr_inf,     // 邻点与当前点存在大的距离跳变（理解为方向与射线方向重合？）
  Nr_blind    // 邻点在盲区，自点实际不在盲区，但也判为在盲区
};

// 输出point时间单位类型的字符串
inline std::string ToTimeUnitString(const int& type) {
  if (type == TIME_UNIT::SEC) return "SEC";
  if (type == TIME_UNIT::MS) return "MS";
  if (type == TIME_UNIT::US) return "US";
  if (type == TIME_UNIT::NS) return "NS";
  return "UnknownTimeUnit";
}

// 输出雷达类型的字符串
inline std::string ToLidarString(const LIDAR_TYPE& type) {
  if (type == LIDAR_TYPE::AVIA) return "AVIA";
  if (type == LIDAR_TYPE::VELO16) return "VELO16";
  if (type == LIDAR_TYPE::OUST64) return "OUST64";
  if (type == LIDAR_TYPE::HESAI128) return "HESAI128";
  return "UnknownLidar";
}

inline std::string ToLidarString(const int& type) {
  if (type == LIDAR_TYPE::AVIA) return "AVIA";
  if (type == LIDAR_TYPE::VELO16) return "VELO16";
  if (type == LIDAR_TYPE::OUST64) return "OUST64";
  if (type == LIDAR_TYPE::HESAI128) return "HESAI128";
  return "UnknownLidar";
}

/// @brief 具体指的是(LiDAR)预处理的类型？
struct OrgType
{
  double range = 0; // 本点的range
  double dista = 0; // 到邻点(通常取next)的距离的平方
  double angle[2];  // 前后邻点的角度
  double intersect;   //向量AB与向量AC的夹角(的cos值)
  FEATURE_TYPE ftype; //当前点的类型
  E_JUMP_TYPE edj[2]; //前后邻点的EdgeJump类型
  OrgType()
  {
    range = 0;
    ftype = Nor; // 默认值
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
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

namespace hesai_pcl {
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

POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_pcl::Point,
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