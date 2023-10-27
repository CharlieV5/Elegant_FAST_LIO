#include "lidar_preprocess.h"

#include <omp.h>

#define RETURN0     0x00
#define RETURN0AND1 0x10

namespace fastlio {
} // namespace fastlio

namespace {

void CheckFieldsAndReport(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // TODO: 检查点云消息的fields有哪些
  return;
}

}

LidarPreprocess::LidarPreprocess(ros::NodeHandle* node_handle)
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), keep_every_n_points(1)
{
  hesai_preproc_type = 1;
  hesai_check_disorders = false;

  N_SCANS   = 6;
  SCAN_RATE = 10;
  group_size = 8;
  disA = 0.01;
  // disA = 0.1; // B?     // wgh已明白这里的含义，至于这个取值是写错了还是故意的，我们后边自己调参吧
  disB = 0.1;
  inf_bound = 10;       // 
  p2l_ratio = 225;      // 对应15.0倍
  maxmid_ratio =6.25;   // 对应2.5倍（针对livox）
  midmin_ratio =6.25;   // 对应2.5倍（针对livox）
  maxmin_ratio = 3.24;  // 对应1.8倍（针对普通lidar）
  jump_up_limit = 170.0;  //
  jump_down_limit = 8.0;  // 
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit/180*M_PI);    //cos(170度)
  jump_down_limit = cos(jump_down_limit/180*M_PI);//cos(8度)
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);

  if (node_handle)
  {
    node_handle_ = node_handle;
    pub_hesai_ring = node_handle_->advertise<std_msgs::Float64>("/hesai_pt_ring", 100000);
    pub_hesai_column = node_handle_->advertise<std_msgs::Float64>("/hesai_pt_column", 100000);
    pub_hesai_time = node_handle_->advertise<std_msgs::Float64>("/hesai_pt_time", 100000);
    pub_enabled = true;
  }
  else
  {
    pub_enabled = false;
  }
}

LidarPreprocess::~LidarPreprocess() {}

void LidarPreprocess::SetMode(bool feat_en, int lid_type, double min_range, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = min_range;
  keep_every_n_points = pfilt_num;
}

void LidarPreprocess::Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{  
  LivoxAviaHandler(msg);
  *pcl_out = pl_surf;
}

void LidarPreprocess::Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  // 总是把输入时间单位（由time_unit指定）转化为ms时间单位
  switch (time_unit)
  {
    case SEC:
      time_unit_scale = 1.e3f;
      break;
    case MS:
      time_unit_scale = 1.f;
      break;
    case US:
      time_unit_scale = 1.e-3f;
      break;
    case NS:
      time_unit_scale = 1.e-6f;
      break;
    default:
      time_unit_scale = 1.f;
      break;
  }

  switch (lidar_type)
  {
  case OUST64:
    Ouster64Handler(msg);
    break;

  case VELO16:
    VelodyneHandler(msg);
    break;

  case HESAI128:
    Hesai128Handler(msg);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

void LidarPreprocess::LivoxAviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg->point_num;
  // std::cout << "plsie: " << plsize << std::endl;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;
  
  if (feature_enabled)
  {
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;
        pl_full[i].y = msg->points[i].y;
        pl_full[i].z = msg->points[i].z;
        pl_full[i].intensity = msg->points[i].reflectivity;
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

        bool is_new = false;
        if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }
    }
    static int count = 0;
    static double time = 0.0;
    count ++;
    double t0 = omp_get_wtime();
    for(int j=0; j<N_SCANS; j++)
    {
      if(pl_buff[j].size() <= 5) continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      std::vector<OrgType> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for(uint i=0; i<plsize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
      }
      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      ExtractRingFeature(pl, types);
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;
    printf("Feature extraction time: %lf \n", time / count);
  }
  else
  {
    for(uint i=1; i<plsize; i++)
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num ++;
        if (valid_num % keep_every_n_points == 0)
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

          // wgh:这里的选择逻辑没太懂。。。。意思是过滤掉过于接近（位置重叠）的点？应该是这个意思
          if( (abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) || 
              (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7) || 
              (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7) && 
              (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)) )
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}

void LidarPreprocess::Ouster64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_pcl::Point> pl_raw;
  pcl::fromROSMsg(*msg, pl_raw);
  int plsize = pl_raw.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range = pl_raw.points[i].x * pl_raw.points[i].x 
                    + pl_raw.points[i].y * pl_raw.points[i].y 
                    + pl_raw.points[i].z * pl_raw.points[i].z;
      if (range < (blind * blind)) continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_raw.points[i].x;
      added_pt.y = pl_raw.points[i].y;
      added_pt.z = pl_raw.points[i].z;
      added_pt.intensity = pl_raw.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_raw.points[i].t * time_unit_scale; // curvature unit: ms
      if(pl_raw.points[i].ring < N_SCANS)
      {
        pl_buff[pl_raw.points[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      std::vector<OrgType> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      ExtractRingFeature(pl, types);
    }
  }
  else /*feature not enabled*/
  {
    double time_stamp = msg->header.stamp.toSec();
    // std::cout << "===================================" << std::endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_raw.points.size(); i++)
    {
      if (i % keep_every_n_points != 0) continue;

      double range = pl_raw.points[i].x * pl_raw.points[i].x 
                    + pl_raw.points[i].y * pl_raw.points[i].y 
                    + pl_raw.points[i].z * pl_raw.points[i].z;
      
      if (range < (blind * blind)) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_raw.points[i].x;
      added_pt.y = pl_raw.points[i].y;
      added_pt.z = pl_raw.points[i].z;
      added_pt.intensity = pl_raw.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_raw.points[i].t * time_unit_scale; // curvature unit: ms

      pl_surf.points.push_back(added_pt);
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void LidarPreprocess::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    static bool fields_checked = false;
    if (!fields_checked) {
      CheckFieldsAndReport(msg);
      fields_checked = true;
    }

    pcl::PointCloud<velodyne_pcl::Point> pl_raw; // original
    pcl::fromROSMsg(*msg, pl_raw);
    int plsize = pl_raw.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;             // scan angular velocity
    std::vector<bool> is_first_pt(N_SCANS,true);    // 是否是某个ring上的第一个点,下边的代码很清楚
    std::vector<double> yaw_first_pt(N_SCANS, 0.0); // yaw of first point
    std::vector<float> yaw_last_pt(N_SCANS, 0.0);   // yaw of last point
    std::vector<float> time_last_pt(N_SCANS, 0.0);  // time offset of last point 
    /*****************************************************************/

    if (pl_raw.points[plsize - 1].time > 0)
    {
      given_offset_time = true;
    }
    else
    {
      /*如果没有给出点的时间戳，则用yaw角反推点的时间戳（需要用到Lidar的型号参数）
      但令我不解的是，yaw_first_pt和yaw_last_pt_pt都是局部变量，外界压根儿用不了，没意义啊*/
      given_offset_time = false;

      /*经分析，我认为下边这段代码是试验代码，已没有意义，还没来得及删除罢了*/
      double yaw_first_pt = atan2(pl_raw.points[0].y, pl_raw.points[0].x) * 57.29578;
      double yaw_last_pt_pt  = yaw_first_pt;
      int layer_first = pl_raw.points[0].ring;
      for (uint i = plsize - 1; i > 0; i--)
      {
        /*这里指的是，从后往前，寻找同一根ring上的最后一个点，并假设点的顺序与实际扫描顺序一致，
        也即第一个点和最后一个点分别代表扫描的两端，由此这两个点可以给出yaw角的范围*/
        if (pl_raw.points[i].ring == layer_first)
        {
          yaw_last_pt_pt = atan2(pl_raw.points[i].y, pl_raw.points[i].x) * 57.29578;
          break;
        }
      }
    }

    if(feature_enabled)
    {
      /* step01: 我们按ring来提取特征，提取前，先重置所有ring上的信息 */
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }

      /* step02: 按ring压入点 */
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        int layer  = pl_raw.points[i].ring;
        if (layer >= N_SCANS) continue;
        added_pt.x = pl_raw.points[i].x;
        added_pt.y = pl_raw.points[i].y;
        added_pt.z = pl_raw.points[i].z;
        added_pt.intensity = pl_raw.points[i].intensity;
        added_pt.curvature = pl_raw.points[i].time * time_unit_scale; // units: ms

        if (!given_offset_time)
        {
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
          if (is_first_pt[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first_pt[layer]);
            yaw_first_pt[layer]=yaw_angle;
            is_first_pt[layer]=false;
            yaw_last_pt[layer]=yaw_angle;
            added_pt.curvature = 0.0;
            time_last_pt[layer]=added_pt.curvature;
            continue;
          }

          if (yaw_angle <= yaw_first_pt[layer])
          {
            added_pt.curvature = (yaw_first_pt[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_first_pt[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last_pt[layer]) { added_pt.curvature+=360.0/omega_l; }

          yaw_last_pt[layer] = yaw_angle;
          time_last_pt[layer]=added_pt.curvature;
        }

        pl_buff[layer].points.push_back(added_pt);
      }

      /* step03: 按ring提取特征 */
      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j];
        int linesize = pl.size();
        if (linesize < 2) continue;
        std::vector<OrgType> &types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--; //防止取到最后一个点(没有next)
        for (uint i = 0; i < linesize; i++)
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          vx = pl[i].x - pl[i + 1].x;
          vy = pl[i].y - pl[i + 1].y;
          vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        ExtractRingFeature(pl, types);
      }
    }
    else /*feature not enabled*/
    {
      for (int i = 0; i < plsize; i++)
      {
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_raw.points[i].x;
        added_pt.y = pl_raw.points[i].y;
        added_pt.z = pl_raw.points[i].z;
        added_pt.intensity = pl_raw.points[i].intensity;
        added_pt.curvature = pl_raw.points[i].time * time_unit_scale;  // curvature unit: ms

        if (!given_offset_time)
        {
          int layer = pl_raw.points[i].ring;
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

          if (is_first_pt[layer])
          {
            // printf("layer: %d; is first: %d", layer, is_first_pt[layer]);
            yaw_first_pt[layer] = yaw_angle;
            is_first_pt[layer] = false;
            yaw_last_pt[layer] = yaw_angle;
            added_pt.curvature = 0.0; /*首点置0*/
            time_last_pt[layer] = added_pt.curvature;
            continue;
          }

          // compute offset time
          if (yaw_angle <= yaw_first_pt[layer])
          {
            added_pt.curvature = (yaw_first_pt[layer]-yaw_angle) / omega_l;
          }
          else
          {
            added_pt.curvature = (yaw_first_pt[layer]-yaw_angle+360.0) / omega_l;
          }

          if (added_pt.curvature < time_last_pt[layer]) { added_pt.curvature += 360.0/omega_l; }

          yaw_last_pt[layer] = yaw_angle;
          time_last_pt[layer]=added_pt.curvature;
        }

        if (i % keep_every_n_points == 0)
        {
          if((added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z) > (blind*blind))
          {
            pl_surf.points.push_back(added_pt);
          }
        }
      }
    }

    std::cout << "[ LidarPreproc::VelodyneHandler ] raw points " << plsize << ", surfs " << pl_surf.size() << std::endl;
}

void LidarPreprocess::Hesai128Handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // debug
  static size_t counts = 0;
  counts++; 
  std::cout << "[ LidarPreproc::Hesai128Handler ] received msg counts: " << counts << std::endl;

  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  static bool fields_checked = false;
  if (!fields_checked) {
    CheckFieldsAndReport(msg);
    fields_checked = true;
  }

  pcl::PointCloud<hesai_pcl::Point> pl_raw; // original
  pcl::fromROSMsg(*msg, pl_raw);
  int plsize = pl_raw.points.size();
  if (plsize == 0) return;
  pl_surf.reserve(plsize);

  // 给定数据结构（通常AT128的分辨率是1200*128）
  const int kNumRings = 128;
  static pcl::PointCloud<hesai_pcl::Point> pl_organized[128];
  static bool hesai_pl_inited = false;
  if (!hesai_pl_inited) {
    for (int i = 0; i<kNumRings; i++) {
      pl_organized[i].reserve(10000);
    }
    hesai_pl_inited = true;
  }

  // 按ring排列当前帧点云
  for (int i = 0; i<kNumRings; i++) {
    pl_organized[i].clear();
    pl_buff[i].clear();
  }
  int num_out_of_ring = 0;
  for (const auto& pt : pl_raw.points) {
    if (pt.ring < 0 || pt.ring > 127) { 
      num_out_of_ring++;
      continue; 
    }
    /*Hesai版本*/
    pl_organized[pt.ring].push_back(pt);
    /*兼容旧版本*/
    PointType added_pt;
    added_pt.normal_x = 0;
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pt.x;
    added_pt.y = pt.y;
    added_pt.z = pt.z;
    added_pt.intensity = pt.intensity;
    added_pt.curvature = pt.time * time_unit_scale; // units: ms
    pl_buff[pt.ring].push_back(added_pt);
  }

  // 统计
  std::cout << "scan has " << plsize << " points, ave " << plsize * 1.0 / kNumRings << " / ring, " 
    << num_out_of_ring << " out of ring, valid point statistics: " << std::endl;
  // for (int i = 0; i<kNumRings; i++) {
  //   std::cout << i << "{" << pl_organized[i].size() << "}, ";
  //   if (i > 0 && ((i+1) % 20 == 0)) { std::cout << std::endl; }
  // }
  // std::cout << std::endl;

  // 乱序检查
  if (hesai_check_disorders) {
    for (int i = 0; i<kNumRings; i++) {
      CheckDisOrderedPts(i, pl_organized[i]);
    }
  }

  // 提取特征
  /** 思考：我们想要什么点？
   * 环境中的平面点，杆状物（交通设施杆），柱状物（包括树木），高反路牌边缘，
   * 
   * 1. 自适应跳点，distance越近、跳点比例越大，distance较远时、不跳点
   * 2. 分扇区处理，无论最终是提取几何特征还是抽点，都要使保留的点尽可能均匀地分布在空间中
   * 3.1 如果是提取几何特征：在每个扇区内，{平面度、edge度、倾角、等}，做计算和排序
   * 3.2 如果是抽点：xxxx
  */
  // std::cout << "work with preproc mode " << hesai_preproc_type << std::endl;
  if (hesai_preproc_type == 1 /*原版:无特征,纯跳点*/) {
    const int kKeepEveryNPts = 10;
    for (auto& ring_points : pl_organized) {
      for (size_t i = 0; i < ring_points.size(); i++) {
        if (i % kKeepEveryNPts == 0 && ring_points.points[i].distance > blind) {
          PointType added_pt;
          added_pt.normal_x = 0;
          added_pt.normal_y = 0;
          added_pt.normal_z = 0;
          added_pt.x = ring_points.points[i].x;
          added_pt.y = ring_points.points[i].y;
          added_pt.z = ring_points.points[i].z;
          added_pt.intensity = ring_points.points[i].intensity;
          added_pt.curvature = ring_points.points[i].time * 1000;  // s->ms(以ms为单位保存时间)
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
  else if (hesai_preproc_type == 2 /*原版:提取特征*/) {
    for (int j = 0; j < 128; j++) {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2) continue;
      std::vector<OrgType> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--; //防止取到最后一个点(没有next)
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      types[linesize].dista = 0;
      // std::cout << "going to process ring " << j << ", pts " << pl.size() << std::endl;
      ExtractRingFeature(pl, types);
    }
  }
  else if (hesai_preproc_type == 3 /*自研:无特征,自适应跳点*/) {
    /** 如果不提特征，只是跳点的话，需要考虑：
     * 目的是让留下的点的空间分布均匀，且保留明显的几何特征
     * 1. range越近的地方，跳点比例越大；越远的地方，跳点比例降低
     * 2. 明显的平面特征，明显的edge特征(平面交线)，明显的杆状特征(路灯杆&交通标志牌边缘)，明显的高反特征
    */
    int kKeepEveryNPts = 10;
    // ...

  }
  else if (hesai_preproc_type == 4 /*自研:提取特征*/) {
    /** 共有128根线，如果总共5000个点，每根线约40个点
     * 如果每根线（120度FOV）分成4个扇区，每个扇区为30度，约10个点
    */
    for (size_t j = 0; j < 128; j++) {
      auto &pl = pl_organized[j];
      int linesize = pl.size();
      if (linesize < 2) continue;
      std::vector<OrgType> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--; //防止取到最后一个点(没有next)
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = pl[i].distance;
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      ExtractHesaiRingFeature(pl, types);
    }
  }
  else {
    std::cout << "error." << std::endl;
  }

  // done.
  std::cout << "[ LidarPreproc::Hesai128Handler ] raw points " << plsize 
    << ", surfs " << pl_surf.size() << ", corners " << pl_corn.size() << std::endl << std::endl;

  // debug with colorizing.
  if (fastlio::options::opt_colorize_output_features) {
    // 若开启着色，用normal_z表示颜色值
    for (auto& pt : pl_surf.points) {
      pt.normal_z = 100;
    }
    for (auto& pt : pl_corn.points) {
      pt.normal_z = 255;
    }
    // for (auto& pt : pl_surf.points) {
    //   pt.intensity = 100;
    // }
    // for (auto& pt : pl_corn.points) {
    //   pt.intensity = 255;
    // }
  } else {
    //
  }
  pl_surf += pl_corn;

}

void LidarPreprocess::ExtractRingFeature(pcl::PointCloud<PointType> &pl, std::vector<OrgType> &types)
{
  /** wgh批注：
   * 输入到这个函数时，每一个点的特征描述中，只给定了OrgType::range和OrgType::dista，
   * 也即特征提取只依赖ring上的排列顺序和这两个信息，来提取feature。
  */

  const int plsize1 = pl.size();  // 总点数，不可变
  int plsize2;                    // 实际考虑的点数，视情况而变
  if(plsize1 == 0 || (pl.size() != types.size()))
  {
    printf("something wrong: empty ring size, exit. \n");
    return;
  }

  uint head = 0; // 有效点的起始位置
  /*官方版真就bug多多,原来的条件只有第一个，此时若整个ring上点都在盲区外，此时head将越界！*/
  while(types[head].range < blind && head < plsize1) /*original bug fixed.*/
  {
    head++;
  }

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0; 
  uint i2;
  uint last_i = 0;      // 上一轮的起始点
  uint last_i_nex = 0;  // 上一轮的结束点
  int last_state = 0;
  int plane_type;

  if (lidar_type == HESAI128) {
    // std::cout << "[hesai128 | extract ring feature] pts " << pl.size() << ", types " << types.size() << std::endl;
  }

  //  ***************************************************************************
  /// ***************************** loop1：搜索面特征 *****************************
  //  ***************************************************************************
  plsize2 = (plsize1 > group_size) ? (plsize1 - group_size) : 0;
  for(uint i=head; i<plsize2; i++)
  {
    if(types[i].range < blind)
    {
      continue;
    }

    i2 = i; //起始位置

    // 判断从索引i开始的连续若干个点是否构成“粗略的”局部{平面}特征，所谓的局部是[i, i_next)这段区间，如果不构成特征，则下次检索从{i_next}开始
    plane_type = group_plane_judge(pl, types, i, i_nex/*output*/, curr_direct/*output*/);
    
    if(plane_type == 1) /*返回值1: 构成有效特征，满足plane_judge函数中的所有条件 */
    {
      for(uint j=i; j<=i_nex; j++)
      { 
        if(j!=i && j!=i_nex)
        {
          types[j].ftype = FEATURE_TYPE::Real_Plane;  //非group起止点，确凿的surf点
        }
        else
        {
          types[j].ftype = FEATURE_TYPE::Poss_Plane;  //group起止点，存疑的surf点，也可能是edge
        }
      }
      
      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1/*只要是1类型，都是非零的*/)
      {
        double mod = last_direct.transpose() * curr_direct; //两个单位向量的内积，等价于cos夹角
        if(mod>-0.707 && mod<0.707) /*如果和上一个direction夹角较大，认定起始点为edge点 */
        {
          types[i].ftype = FEATURE_TYPE::Edge_Plane;  //应该是两个plane形成的edge上的点
        }
        else
        {
          types[i].ftype = FEATURE_TYPE::Real_Plane;  //两个plane近似平行，则认定相交处的点也是surf点
        }
      }
      
      i = i_nex - 1;  //设置下一轮的起点
      last_state = 1; //当然是1了
    }
    else // if(plane_type == 2) /*返回值2: 不构成典型特征 */
    {
      i = i_nex;      //下次检索从i_next开始
      last_state = 0; //记为无效
    }

    // else if(plane_type == 0) /*返回值0: xx特征 */
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;
    //       uint ttem = group_plane_judge(pl, types, j, i_nex_tem, curr_direct2);
    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }
    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = FEATURE_TYPE::Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = FEATURE_TYPE::Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }
    //   }
    // }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }
  if (lidar_type == HESAI128) {
    // std::cout << "[hesai128 | extract ring feature] search surfs done. " << std::endl;
  }

  //  ***************************************************************************
  /// ************************** loop2：大概是搜索edge特征 **************************
  //  ***************************************************************************
  plsize2 = plsize1 > 3 ? plsize1 - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i].ftype>=FEATURE_TYPE::Real_Plane)
    {
      continue;
    }

    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    // LiDAR原点记为O，当前点记为A，左右邻点记为B,C
    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z); //向量OA
    Eigen::Vector3d vecs[2];  //向量OB和OC

    // 判断前后邻点的edge类型，这将用于下一步判断自点的feature类型
    for(int j=0; j<2; j++)
    {
      int m = -1;
      if(j == 1)
      {
        m = 1;
      }

      // 若邻点在盲区
      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound/*10m*/)
        {
          types[i].edj[j] = Nr_inf;   //跳变较远
        }
        else
        {
          types[i].edj[j] = Nr_blind; //也认为是盲区点
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z); //OB（或OC）
      vecs[j] = vecs[j] - vec_a;  //AB（或AC）
      
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm(); //AB与OA的夹角（的cos值）
      if(types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;   //邻点在OA上，也即180度夹角
      }
      else if(types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;  //邻点在OA延长线上，也即0度夹角
      }
    }

    // 
    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = FEATURE_TYPE::Edge_Jump;
        }
      }
    }
    //
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = FEATURE_TYPE::Edge_Jump;
        }
      }
    }
    //
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = FEATURE_TYPE::Edge_Jump;
      }
    }
    //
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = FEATURE_TYPE::Edge_Jump;
      }
     
    }
    //
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == FEATURE_TYPE::Nor)
      {
        types[i].ftype = FEATURE_TYPE::Wire;
      }
    }
  }
  if (lidar_type == HESAI128) {
    // std::cout << "[hesai128 | extract ring feature] search edges done. " << std::endl;
  }

  //  ***************************************************************************
  /// ********************** loop3：大概是收尾一些被漏掉的特征？ **********************
  //  ***************************************************************************
  plsize2 = plsize1-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    if(types[i].ftype == FEATURE_TYPE::Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == FEATURE_TYPE::Nor)
        {
          types[i-1].ftype = FEATURE_TYPE::Real_Plane;
        }
        if(types[i+1].ftype == FEATURE_TYPE::Nor)
        {
          types[i+1].ftype = FEATURE_TYPE::Real_Plane;
        }
        types[i].ftype = FEATURE_TYPE::Real_Plane;
      }
    }
  }
  if (lidar_type == HESAI128) {
    // std::cout << "[hesai128 | extract ring feature] re-search features done. " << std::endl;
  }

  //  ***************************************************************************
  /// ******************* loop4：保存 {surf & corner} 到全局容器 *******************
  //  ***************************************************************************
  int num_surfs = 0, num_edges = 0;
  int last_surface = -1;
  for(uint j=head; j<plsize1; j++)
  {
    if(types[j].ftype==FEATURE_TYPE::Poss_Plane || types[j].ftype==FEATURE_TYPE::Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }
    
      if(j == uint(last_surface+keep_every_n_points-1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);
        num_surfs++;

        last_surface = -1;
      }
    }
    else
    {
      if(types[j].ftype==FEATURE_TYPE::Edge_Jump || types[j].ftype==FEATURE_TYPE::Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
        num_edges++;
      }
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.intensity /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
        num_surfs++;
      }
      last_surface = -1;
    }
  }
  // std::cout << "[hesai128 | extract ring feature] save features done, surfs " << num_surfs << ", edges " << num_edges << std::endl;

}

int LidarPreprocess::group_plane_judge(const PointCloudXYZI &pl, std::vector<OrgType> &types, const uint i_cur, uint &i_nex, Eigen::Vector3d &group_direct)
{
  /** 在尝试分析了整个函数体后，我们推测 —— 2可能代表【不构成特征】，1可能代表【面特征】，0可能代表【线特征】 */
  /// NOTE: 无论如何i_nex都要输出，下文需要确保其有效性

  // 感觉是指定一个group允许的最大距离范围，只要在此范围内，都可纳入同一个group，即使数量超过groupSize
  double group_dist_limit = disA * types[i_cur].range + disB;
  group_dist_limit = group_dist_limit * group_dist_limit;

  double dist_sq;
  std::vector<double> distSqArray_;
  distSqArray_.reserve(20); // 默认group不超过20，鉴于在groupSize之外也会纳入点到group，这可能有问题  @TODO

  /// ********* loop1: 在groupSize范围内，搜索有效点，并保存distSq队列
  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      group_direct.setZero();
      return 2; //存在盲区点,无法拟合有效平面,直接返回无效类型
    }
    distSqArray_.push_back(types[i_nex].dista);
  }

  /// ********* loop2: 看看后边的点是否满足“与group起点的距离在阈值之内”，若满足则纳入group中，并保存distSq队列
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      group_direct.setZero();
      return 2; ////存在盲区点,无法拟合有效平面,直接返回无效类型
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    dist_sq = vx*vx + vy*vy + vz*vz;
    if(dist_sq >= group_dist_limit)
    {
      break;
    }
    distSqArray_.push_back(types[i_nex].dista);
    i_nex++;
  }

  /// ********* loop3: 在实际的group范围内，从起点到终点拉一条线，计算所有点到这条线的最大距离
  // note: 截止此时，[vx,vy,vz]代表group起点到group终点的向量，dist_sq代表这个向量的模长平方
  double max_p2l_area_sq = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1]*vz - vy*v1[2]; //这里是计算外积，其模长代表面积
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2]; //面积的平方
    if(lw > max_p2l_area_sq)
    {
      max_p2l_area_sq = lw;
    }
  }

  /// ********* 如果group内的点的分布构成长条状{长宽比大于15}，则认为是非平面
  // if((dist_sq*dist_sq/max_p2l_area_sq) < p2l_ratio /*225*/) //这里他妈的有问题吧？？
  if((dist_sq*dist_sq/max_p2l_area_sq) > p2l_ratio /*225*/) //改成大于号
  {
    group_direct.setZero();
    return 0; // 非平面特征（长宽比大于15,近似线特征）
  }

  /// ********* 实现了一个排序算法，将distSq降序排序
  const uint arraySize = distSqArray_.size();
  for(uint j=0; j<arraySize-1; j++)
  {
    for(uint k=j+1; k<arraySize; k++)
    {
      if(distSqArray_[j] < distSqArray_[k])
      {
        double value = distSqArray_[j];
        distSqArray_[j] = distSqArray_[k];
        distSqArray_[k] = value;
      }
    }
  }


  /// 排除可能的数值异常：倒二这个距离在下文要当分母，因此避免它太小造成数值不稳定
  if(distSqArray_[distSqArray_.size()-2] < 1e-16)
  {
    group_direct.setZero();
    return 0;
  }

  /// ********* group中各个点之间应该是连续且均匀的，若不连续（ratio过大），则认定不构成有效平面
  if(lidar_type==AVIA)
  {
    double max_mid_ratio_sq = distSqArray_[0] / distSqArray_[arraySize/2];
    double mid_min_ratio_sq = distSqArray_[arraySize/2] / distSqArray_[arraySize-2];

    if(max_mid_ratio_sq >= maxmid_ratio || mid_min_ratio_sq >= midmin_ratio)
    {
      group_direct.setZero();
      return 0;
    }
  }
  else /*not livox*/
  {
    double max_min_ratio_sq = distSqArray_[0] / distSqArray_[arraySize-2];
    if(max_min_ratio_sq >= maxmin_ratio)
    {
      group_direct.setZero();
      return 0;
    }
  }

  /// ********* 有效平面,返回1
  group_direct << vx, vy, vz;
  group_direct.normalize();
  return 1;
}

bool LidarPreprocess::edge_jump_judge(const PointCloudXYZI &pl, std::vector<OrgType> &types, uint i, SURROUND_TYPE nor_dir)
{
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

 
  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }
  
  return true;
}

void LidarPreprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct, ros::Publisher& publisher, const LIDAR_TYPE &type)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = ToLidarString(type);
  output.header.stamp = ct;
  publisher.publish(output);
}

void LidarPreprocess::CheckDisOrderedPts(const int& i, const pcl::PointCloud<hesai_pcl::Point>& ring_points)
{
  double last_pt_time = -1e5, last_pt_column = -1e5;
  int time_rewind_cnts = 0, column_rewind_cnts = 0;
  const size_t ring_size = ring_points.size();

  // check timestamp
  for (size_t j = 0; j < ring_size; ++j) {
    const auto& pt = ring_points[j];
    if (pt.time < last_pt_time) {
      std::cout << "    ring " << i << ", pt " << (j+1) << "/" << ring_size << ", time rewind back " << pt.time - last_pt_time;
      time_rewind_cnts++;
      continue;
    }
    last_pt_time = pt.time;
  }
  if (time_rewind_cnts > 0) {
    std::cout << " | ring " << i << ", time rewind counts " << time_rewind_cnts << std::endl;
  }

  // check column
  for (size_t j = 0; j < ring_size; ++j) {
    const auto& pt = ring_points[j];
    if (pt.column < last_pt_column) {
      std::cout << "    ring " << i << ", pt " << (j+1) << "/" << ring_size << ", column rewind back " << pt.column - last_pt_column;
      column_rewind_cnts++;
      continue;
    }
    last_pt_column = pt.column;
  }
  if (column_rewind_cnts > 0) {
    std::cout << " | ring " << i << ", column rewind counts " << column_rewind_cnts << std::endl;
  }

}

void LidarPreprocess::ExtractHesaiRingFeature(pcl::PointCloud<hesai_pcl::Point> &pl, std::vector<OrgType> &types)
{
  //
  int linesize = pl.size();
  if (linesize < 10) {
    //

    return;
  }
}



