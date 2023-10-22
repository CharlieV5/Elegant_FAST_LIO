#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <livox_ros_driver/CustomMsg.h>

#include "common/options.h"
#include "common/sensor_lidar_type.h"

namespace fastlio {
} // namespace fastlio


/// @brief 鉴于提取特征点的算法和Lidar的扫描方式紧密相关，因此预处理部分最好放到ros层，这样算法层就和lidar类型解耦了
class LidarPreprocess {
  public:

  LidarPreprocess();
  ~LidarPreprocess();
  
  void Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void SetMode(bool feat_en, int lid_type, double bld, int pfilt_num);

  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128];      //maximum 128 line lidar, 用于辅助提取feature, 按ring保存点，单个ring上的点按先后顺序压入
  std::vector<OrgType> typess[128]; //maximum 128 line lidar, 用于辅助提取feature, 按ring记录点的特征信息，单个ring上按先后顺序压入
  int time_unit;            // 见枚举类型
  float time_unit_scale;    // 
  int lidar_type;           // 见枚举类型
  int N_SCANS, SCAN_RATE;   // 
  int keep_every_n_points;  // 每N个点保存一个点，设置为1意味着全部保存
  double blind;             // 盲区距离，也即小于此距离的点因过近而被丢掉
  bool feature_enabled;     // 
  bool given_offset_time;   // 原始消息中是否给出了点的相对时间戳
  ros::Publisher pub_full, pub_surf, pub_corn;


  private:
  void LivoxAviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void LiDAR128Handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {/*TODO*/}
  void Ouster64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /// 提取给定ring上的features, 然后压入{pl_surf}中！
  void ExtractRingFeature(PointCloudXYZI &pl /*ring points*/, std::vector<OrgType> &types /*points label*/);

  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, std::vector<OrgType> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, std::vector<OrgType> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, std::vector<OrgType> &types, uint i, SURROUND_TYPE nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
