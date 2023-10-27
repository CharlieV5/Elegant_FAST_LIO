
#include "imu_process.h"


const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)
{
  init_imu_num_ = 1;
  Q = process_noise_cov();
  cov_acc_esti  = V3D(0.1, 0.1, 0.1);
  cov_gyro_esti = V3D(0.1, 0.1, 0.1);
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);
  cov_bias_gyro = V3D(0.0001, 0.0001, 0.0001);
  mean_acc      = V3D(0, 0, -1.0);
  mean_gyr      = V3D(0, 0, 0);
  gyro_last     = Zero3d;
  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;
  last_imu_obs_.reset(new fastlio::ImuData());
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  std::cout << "Reset ImuProcess." << std::endl;
  mean_acc         = V3D(0, 0, -1.0);
  mean_gyr         = V3D(0, 0, 0);
  gyro_last        = Zero3d;
  imu_need_init_   = true;
  start_timestamp_ = -1;
  init_imu_num_    = 1;
  imu_poses_.clear();
  last_imu_obs_.reset(new fastlio::ImuData());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);
}

void ImuProcess::set_extrinsic(const V3D &transl)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU.setIdentity();
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)
{
  Lidar_T_wrt_IMU = transl;
  Lidar_R_wrt_IMU = rot;
}

void ImuProcess::set_gyr_cov(const V3D &scaler)
{
  cov_gyro_fixed = scaler;
}

void ImuProcess::set_acc_cov(const V3D &scaler)
{
  cov_acc_fixed = scaler;
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)
{
  cov_bias_gyro = b_g;
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)
{
  cov_bias_acc = b_a;
}

bool ImuProcess::InitWithIMUs(const MeasureGroup &meas, IeskfomType &kf_state, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu_queue.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu_queue.front()->angular_velocity;
    mean_acc << imu_acc.x(), imu_acc.y(), imu_acc.z();
    mean_gyr << gyr_acc.x(), gyr_acc.y(), gyr_acc.z();
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto &imu : meas.imu_queue)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x(), imu_acc.y(), imu_acc.z();
    cur_gyr << gyr_acc.x(), gyr_acc.y(), gyr_acc.z();

    // 求加权均值，相当于一个平滑均值
    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    // 猜测：假设初始化过程中，载体是静止的，于是下边可以估算出IMU {加加速度&陀螺仪} 的协方差
    cov_acc_esti = cov_acc_esti * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyro_esti = cov_gyro_esti * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    // std::cout << "acc norm: " << cur_acc.norm() << " " << mean_acc.norm() << std::endl;

    N++;
  }
  std::cout << "[Imu Process] Initialization: acc mean " << mean_acc.transpose() << ", acc cov " << cov_acc_esti.transpose() << std::endl;
  std::cout << "[Imu Process] Initialization: gyr mean " << mean_gyr.transpose() << ", gyr cov " << cov_gyro_esti.transpose() << std::endl;

  // 初始化滤波器的初始状态
  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2); /*静止假设&估算初始重力方向；重力向量是单位向量,可被视作S2流形*/
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;  /*静止假设下，陀螺仪的观测可以被作为bias*/
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;  /*保存外参，合理*/
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;

  // 更新滤波器状态
  kf_state.change_x(init_state);

  // 初始化滤波器的协方差
  IeskfomType::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;
  init_P(21,21) = init_P(22,22) = 0.00001; 

  // 设置协方差
  kf_state.change_P(init_P);

  // 其它
  last_imu_obs_ = meas.imu_queue.back();

  return true;

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, IeskfomType &kf_state, PointCloudXYZI &pcl_out)
{
  /// 把上一次处理的最后一帧IMU观测加进来，这会使姿态估算更有效？
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu_queue_ = meas.imu_queue;
  v_imu_queue_.push_front(last_imu_obs_);
  
  const double &imu_beg_time = v_imu_queue_.front()->timestamp;
  const double &imu_end_time = v_imu_queue_.back()->timestamp;
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;
  
  /// 点云中的点，按时间重新排序
  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // std::cout << "[ IMU Process ]: Process lidar from " << pcl_beg_time << " to " << pcl_end_time << ", " \
  //          << meas.imu.size() << " imu msgs from " << imu_beg_time << " to " << imu_end_time << std::endl;

  /// 初始第一帧IMU位姿
  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();
  imu_poses_.clear();
  imu_poses_.push_back(ToPose6D(0.0, acc_s_last, gyro_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));

  /// 正向传播：IMU姿态积分，和置信度(协方差矩阵)估算，等
  /*** forward propagation at each imu point ***/
  V3D gyro_ave, acc_ave;
  double dt = 0;
  input_ikfom obs_in;
  for (auto it_imu = v_imu_queue_.begin(); it_imu < (v_imu_queue_.end() - 1); it_imu++)
  {
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    
    if (tail->timestamp < last_lidar_end_time_) { continue; }
    
    gyro_ave << 0.5 * (head->angular_velocity.x() + tail->angular_velocity.x()),
                0.5 * (head->angular_velocity.y() + tail->angular_velocity.y()),
                0.5 * (head->angular_velocity.z() + tail->angular_velocity.z());
    acc_ave  << 0.5 * (head->linear_acceleration.x() + tail->linear_acceleration.x()),
                0.5 * (head->linear_acceleration.y() + tail->linear_acceleration.y()),
                0.5 * (head->linear_acceleration.z() + tail->linear_acceleration.z());

    // fout_imu << std::setw(10) << head->timestamp - first_lidar_time << " " << gyro_ave.transpose() << " " << acc_ave.transpose() << std::endl;

    acc_ave = acc_ave * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

    if(head->timestamp < last_lidar_end_time_)
    {
      dt = tail->timestamp - last_lidar_end_time_;
      // dt = tail->timestamp - pcl_beg_time;
    }
    else
    {
      dt = tail->timestamp - head->timestamp;
    }
    
    obs_in.acc = acc_ave;
    obs_in.gyro = gyro_ave;
    Q.block<3, 3>(0, 0).diagonal() = cov_gyro_esti;
    Q.block<3, 3>(3, 3).diagonal() = cov_acc_esti;
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyro;
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;
    kf_state.predict(dt, Q, obs_in);

    /* save the poses at each IMU measurements */
    // 保存每一帧的积分结果
    imu_state = kf_state.get_x();
    gyro_last = gyro_ave - imu_state.bg;
    acc_s_last  = imu_state.rot * (acc_ave - imu_state.ba);
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];
    }
    double &&offs_t = tail->timestamp - pcl_beg_time;
    imu_poses_.push_back(ToPose6D(offs_t, acc_s_last, gyro_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));
  }

  /// 正向传播的最后一次积分 (接近点云帧尾时刻)
  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q, obs_in);
  
  imu_state = kf_state.get_x();
  last_imu_obs_ = meas.imu_queue.back();
  last_lidar_end_time_ = pcl_end_time;

  /// 反向传播：点云运动补偿
  /*** undistort each lidar point (backward propagation) ***/
  V3D acc_imu, vel_imu, pos_imu;
  M3D R_imu;
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = imu_poses_.end() - 1; it_kp != imu_poses_.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu << MAT_FROM_ARRAY(head->rot);
    // std::cout << "head imu acc: " << acc_imu.transpose() << std::endl;
    vel_imu << VEC_FROM_ARRAY(head->vel);
    pos_imu << VEC_FROM_ARRAY(head->pos);
    acc_imu << VEC_FROM_ARRAY(tail->acc);
    gyro_ave << VEC_FROM_ARRAY(tail->gyr);

    for(; it_pcl->curvature / double(1000)/*时间单位回到秒*/ > head->offset_time; it_pcl --)
    {
      dt = it_pcl->curvature / double(1000)/*时间单位回到秒*/ - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation //运动补偿:只使用了旋转
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      M3D R_i(R_imu * Exp(gyro_ave, dt));
      
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * 
        (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!
      
      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

bool ImuProcess::Process(const MeasureGroup &meas,  IeskfomType &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1,t2,t3;
  t1 = omp_get_wtime();

  // 准入
  if(meas.imu_queue.empty()) 
  {
    std::cout << "measurements' imu queue empty, failed to process measures, exit." << std::endl;
    return false; 
  }
  assert(meas.lidar != nullptr);

  static size_t process_cnts = 0;
  process_cnts++;
  std::cout << "[ Imu Process ] process with imu counts " << meas.imu_queue.size() 
    << ", seq=" << process_cnts << std::endl;

  /// 初始化IMUProc
  if (imu_need_init_)
  {
    /// The very first lidar frame
    const bool inited = InitWithIMUs(meas, kf_state, init_imu_num_);

    imu_need_init_ = true;

    last_imu_obs_ = meas.imu_queue.back();

    state_ikfom imu_state = kf_state.get_x();
    if (init_imu_num_ > fastlio::options::opt_min_imu_for_init)
    {
      cov_acc_esti *= pow(G_m_s2 / mean_acc.norm(), 2);
      imu_need_init_ = false;

      // 搞不懂，Init函数里估算了cov，但这里又重置为了人为设定的cov值
      cov_acc_esti = cov_acc_fixed;
      cov_gyro_esti = cov_gyro_fixed;
      std::cout << "[ Imu Process ] Initialization Done." << std::endl;
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyro[0], cov_bias_gyro[1], cov_bias_gyro[2], \
      //          cov_acc_esti[0], cov_acc_esti[1], cov_acc_esti[2], cov_gyro_esti[0], cov_gyro_esti[1], cov_gyro_esti[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),std::ios::out);
    }

    return false;
  }

  /// 用IMU姿态积分，做点云运动补偿(仅姿态)
  UndistortPcl(meas, kf_state, *cur_pcl_un_);

  t2 = omp_get_wtime();
  t3 = omp_get_wtime();
  
  // std::cout << "[ IMU Process ]: Time: " << t3 - t1 << std::endl;

  return true;
}