#include <ros/ros.h>
#include <std_msgs/Float64.h>
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

  LidarPreprocess(ros::NodeHandle* node_handle = nullptr);
  ~LidarPreprocess();
  
  void Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void SetMode(bool feat_en, int lid_type, double min_range, int pfilt_num);

  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128];      //maximum 128 line lidar, 用于辅助提取feature, 按ring保存点，单个ring上的点按先后顺序压入
  std::vector<OrgType> typess[128]; //maximum 128 line lidar, 用于辅助提取feature, 按ring记录点的特征信息，单个ring上按先后顺序压入
  int time_unit;            // 见枚举类型，最终的目的是，pt的时间戳必须用【ms】表达，运动补偿时再转换回【s】，推测是为了避免数值损失？
  float time_unit_scale;    // 
  int lidar_type;           // 见枚举类型
  int N_SCANS, SCAN_RATE;   // 
  int keep_every_n_points;  // 每N个点保存一个点，设置为1意味着全部保存

  double blind;             // 盲区距离，也即小于此距离的点因过近而被丢掉
  double inf_bound;         // 没搞明白是干啥的？
  
  bool feature_enabled;     // 
  bool given_offset_time;   // 原始消息中是否给出了点的相对时间戳
  bool pub_enabled = false;
  ros::NodeHandle* node_handle_ = nullptr;
  ros::Publisher pub_full, pub_surf, pub_corn;
  ros::Publisher pub_hesai_ring, pub_hesai_column, pub_hesai_time;

  int hesai_preproc_type;       // 仅针对Hesai雷达
  bool hesai_check_disorders;   // 仅针对Hesai雷达

  private:
  void LivoxAviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void Hesai128Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void Ouster64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /// 提取给定ring上的features, 然后压入{pl_surf, pl_corn}中！
  void ExtractRingFeature(PointCloudXYZI &pl /*ring points*/, std::vector<OrgType> &types /*points label*/);

  int  group_plane_judge(const PointCloudXYZI &pl, std::vector<OrgType> &types, const uint i_cur, uint &i_nex, Eigen::Vector3d &group_direct);
  bool small_plane(const PointCloudXYZI &pl, std::vector<OrgType> &types, const uint i_cur, uint &i_nex, Eigen::Vector3d &group_direct); /*无效函数*/
  bool edge_jump_judge(const PointCloudXYZI &pl, std::vector<OrgType> &types, uint i, SURROUND_TYPE nor_dir);

  void pub_func(PointCloudXYZI &pl, const ros::Time &ct, ros::Publisher& publisher, const LIDAR_TYPE &type = LIDAR_TYPE::VELO16);

  int group_size;       // N个点组成一个group，作为提取Surf/Corner等特征的单元
  double disA, disB;    // 用于计算group的范围阈值
  double maxmid_ratio, midmin_ratio;  // 针对Livox，判断无效group的阈值
  double maxmin_ratio;                // 针对其它Lidar，判断无效group的阈值
  double p2l_ratio;                   // 点到线的距离阈值，需要大于这个值才能判断组成面
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;  // 判断edge特征的阈值
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;

  /// @brief [Hesai128] 检查hesai激光点云某个ring上是否有乱序的点
  void CheckDisOrderedPts(const int& i, const pcl::PointCloud<hesai_pcl::Point>& ring_points);

  /// @brief [Hesai128] 提取给定ring上的features, 然后压入{pl_surf, pl_corn}中！
  void ExtractHesaiRingFeature(pcl::PointCloud<hesai_pcl::Point> &pl /*ring points*/, std::vector<OrgType> &types /*points label*/);
  
};
