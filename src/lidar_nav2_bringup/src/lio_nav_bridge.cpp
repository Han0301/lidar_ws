#include "lidar_nav2_bringup/lio_nav_bridge.hpp"

namespace lidar_nav2_bringup
{
namespace
{
// 封装 child 在 parent 中的平移和旋转，保留输入扫描的时间戳
geometry_msgs::msg::TransformStamped make_transform
(
  const std::string & parent,
  const std::string & child,
  const builtin_interfaces::msg::Time & stamp,
  const Eigen::Vector3d & position,
  const Eigen::Quaterniond & rotation
)
{
  geometry_msgs::msg::TransformStamped output; // 带父子坐标系和扫描时间的 TF 消息
  output.header.frame_id = parent;
  output.child_frame_id = child;
  output.header.stamp = stamp;
  output.transform.translation.x = position.x();
  output.transform.translation.y = position.y();
  output.transform.translation.z = position.z();
  output.transform.rotation.x = rotation.x();
  output.transform.rotation.y = rotation.y();
  output.transform.rotation.z = rotation.z();
  output.transform.rotation.w = rotation.w();
  return output;
}
}

// 建立重力初始化、LIO 位姿、地面参考输入与导航 TF/里程计输出
LioNavBridge::LioNavBridge() : Node("lio_nav_bridge")
{
  // 读取初始化、接收超时、地面跳变门限和 LiDAR 平移外参
  required_samples_ = declare_parameter<int>("gravity_samples", 100);
  input_timeout_ = declare_parameter<double>("input_timeout_s", 0.6);
  max_ground_innovation_ = declare_parameter<double>("max_ground_innovation_m", 0.5);
  const auto translation = declare_parameter<std::vector<double>>("lidar_translation_in_body",
    std::vector<double>(3, 0.0)); // LiDAR 原点在 body 中的 XYZ 平移（m）
  // 检查外参维度与基本阈值，配置非法时直接拒绝启动
  if (translation.size() != 3 || required_samples_ < 10 || max_ground_innovation_ <= 0.0)
  {
    throw std::invalid_argument("Invalid navigation frame initialization parameters");
  }
  sensor_translation_ = Eigen::Vector3d(translation[0], translation[1], translation[2]); // LiDAR 原点在 body 坐标中的位置（m）
  // 固定参考只发布一次；随 LIO 更新的参考由动态广播器发布
  broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/livox/imu", rclcpp::SensorDataQoS(),
    std::bind(&LioNavBridge::initialize_reference, this, std::placeholders::_1));
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>("/Odometry", 20,
    std::bind(&LioNavBridge::project_pose, this, std::placeholders::_1));
  ground_sub_ = create_subscription<geometry_msgs::msg::PointStamped>("/perception/ground_reference", 3,
    std::bind(&LioNavBridge::ground_reference, this, std::placeholders::_1));
  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("/nav/odom", 20);
  health_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/nav/diagnostics", 10);
  health_timer_ = create_wall_timer(std::chrono::milliseconds(500),
    std::bind(&LioNavBridge::publish_health, this));
}

// 只在启动阶段累计低角速度 IMU 样本，建立固定重力对齐旋转
void LioNavBridge::initialize_reference(sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  // 初始化完成后不再用瞬时加速度逐帧改写重力方向
  if (initialized_)
  {
    return;
  }
  const Eigen::Vector3d acceleration(message->linear_acceleration.x, message->linear_acceleration.y, message->linear_acceleration.z); // IMU 测得的加速度向量，用于初始化方向
  const Eigen::Vector3d angular(message->angular_velocity.x, message->angular_velocity.y, message->angular_velocity.z); // IMU 三轴角速度（rad/s），用于排除明显旋转样本
  // 筛掉无效加速度和明显旋转样本；低角速度不证明不存在平移加速度
  if (!acceleration.allFinite() || acceleration.norm() < 0.3 || angular.norm() > 0.12)
  {
    return;
  }
  acceleration_sum_ += acceleration;
  ++initialization_samples_;
  if (initialization_samples_ < required_samples_)
  {
    return;
  }
  // 把初始化加速度均值方向旋转到 Z 轴，建立 camera_init→odom 固定参考
  level_from_camera_ = Eigen::Quaterniond::FromTwoVectors(
    acceleration_sum_.normalized(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::vector<geometry_msgs::msg::TransformStamped> transforms; // 本次需要一起发布的坐标变换集合
  // map→odom 目前是单位变换，不包含全局重定位或漂移修正
  transforms.push_back(make_transform("map", "odom", message->header.stamp,
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()));
  transforms.push_back(make_transform("odom", "camera_init", message->header.stamp,
    Eigen::Vector3d::Zero(), Eigen::Quaterniond(level_from_camera_)));
  static_broadcaster_->sendTransform(transforms);
  initialized_ = true;
  RCLCPP_INFO(get_logger(), "Gravity reference initialized from %d stationary IMU samples", initialization_samples_);
}

// 保留 LIO 位姿并生成重力对齐后的平面导航参考、传感器 TF 和差分速度
void LioNavBridge::project_pose(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  // 重力未初始化或输入坐标不符合约定时，不生成导航位姿
  if (!initialized_ || message->header.frame_id != "camera_init" || message->child_frame_id != "body")
  {
    return;
  }
  const auto & p = message->pose.pose.position; // FAST-LIO2 输出的 body 原点在 camera_init 中的位置（m）
  const auto & q = message->pose.pose.orientation; // FAST-LIO2 输出的 body→camera_init 旋转四元数
  const Eigen::Quaterniond camera_from_body(q.w, q.x, q.y, q.z); // Eigen 使用 w/x/y/z 顺序，后面校验并归一化
  if (camera_from_body.norm() < 0.5)
  {
    return;
  }
  // 将 LIO 的 camera_init 位姿转到重力对齐 odom，并提取水平航向
  const Eigen::Matrix3d world_from_body = level_from_camera_ * camera_from_body.normalized().toRotationMatrix(); // body→重力对齐 odom 的完整旋转
  const Eigen::Vector3d position = level_from_camera_ * Eigen::Vector3d(p.x, p.y, p.z); // body 原点在 odom 中的三维位置（m）
  const double yaw = std::atan2(world_from_body(1, 0), world_from_body(0, 0)); // 重力对齐后 body X 轴的水平航向（rad）
  const Eigen::Quaterniond yaw_rotation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())); // 只保留航向的旋转，消除手持俯仰和侧倾
  Eigen::Vector3d base_position = position; // 虚拟 nav_base 使用 LIO 的 X/Y，Z 改成滤波地面高度
  base_position.z() = ground_height_.height();
  std::vector<geometry_msgs::msg::TransformStamped> transforms; // 本次需要一起发布的坐标变换集合
  // nav_base 在参考地面上；perception_sensor 保持 body 高度但调平；nav_sensor 保留真实传感器位姿
  transforms.push_back(make_transform("odom", "nav_base", message->header.stamp, base_position, yaw_rotation));
  transforms.push_back(make_transform("odom", "perception_sensor", message->header.stamp, position, yaw_rotation));
  transforms.push_back(make_transform("odom", "nav_sensor", message->header.stamp,
    position + world_from_body * sensor_translation_, Eigen::Quaterniond(world_from_body)));
  broadcaster_->sendTransform(transforms);
  // 封装虚拟导航位姿；该参考不是实际底盘几何或轮式里程计
  nav_msgs::msg::Odometry output; // 发布到 /nav/odom 的虚拟平面参考位姿
  output.header = message->header;
  output.header.frame_id = "odom";
  output.child_frame_id = "nav_base";
  output.pose.pose.position.x = base_position.x();
  output.pose.pose.position.y = base_position.y();
  output.pose.pose.position.z = base_position.z();
  output.pose.pose.orientation.x = yaw_rotation.x();
  output.pose.pose.orientation.y = yaw_rotation.y();
  output.pose.pose.orientation.z = yaw_rotation.z();
  output.pose.pose.orientation.w = yaw_rotation.w();
  // 将 LIO 协方差按固定重力旋转变换到 odom
  Eigen::Matrix<double, 6, 6> covariance; // 原始三维位置/姿态协方差的 6×6 矩阵
  for (int row = 0; row < 6; ++row)
  {
    for (int col = 0; col < 6; ++col)
    {
      covariance(row, col) = message->pose.covariance[row * 6 + col];
    }
  }
  Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero(); // 仅将两个三维协方差块旋转到 odom；不完整建模平面投影的不确定性
  jacobian.block<3, 3>(0, 0) = level_from_camera_;
  jacobian.block<3, 3>(3, 3) = level_from_camera_;
  covariance = jacobian * covariance * jacobian.transpose();
  for (int row = 0; row < 6; ++row)
  {
    for (int col = 0; col < 6; ++col)
    {
      output.pose.covariance[row * 6 + col] = covariance(row, col);
    }
  }
  const double stamp = rclcpp::Time(message->header.stamp).seconds(); // 当前扫描对应的 ROS 消息时间（s）
  const double duration = stamp - previous_stamp_; // 与上一个有效位姿的时间间隔（s）
  // 仅对合理扫描间隔做差分，避免零间隔或大跳时产生异常速度
  if (odometry_received_ && duration > 0.02 && duration < 0.30)
  {
    const Eigen::Vector3d velocity = yaw_rotation.inverse() * ((position-previous_position_)/duration); // 位置差分速度投影到仅含航向的参考坐标（m/s）
    output.twist.twist.linear.x = velocity.x();
    output.twist.twist.linear.y = velocity.y();
    // 航向差先绕回一个周期，避免跨越正负 π 时出现速度尖峰
    output.twist.twist.angular.z = std::remainder(yaw-previous_yaw_, 2*M_PI)/duration;
  }
  odometry_pub_->publish(output);
  previous_position_ = position;
  previous_yaw_ = yaw;
  previous_stamp_ = stamp;
  // 保留最近二十帧位置，给延迟到达的地面参考按扫描时间配对
  pose_history_.emplace_back(stamp, position);
  while (pose_history_.size() > 20)
  {
    pose_history_.pop_front();
  }
  odometry_received_ = true;
  last_odometry_ = std::chrono::steady_clock::now();
}

// 用匹配扫描时刻的位姿，把感知相对高度转成 odom 中的地面高度
void LioNavBridge::ground_reference(geometry_msgs::msg::PointStamped::ConstSharedPtr message)
{
  if (message->header.frame_id != "perception_sensor" || !std::isfinite(message->point.z))
  {
    return;
  }
  const double stamp = rclcpp::Time(message->header.stamp).seconds(); // 当前扫描对应的 ROS 消息时间（s）
  // 从最新位置向前寻找，时间差小于 20 ms 才接受该地面参考
  for (auto it = pose_history_.rbegin(); it != pose_history_.rend(); ++it)
  {
    if (std::abs(it->first - stamp) < 0.02)
    {
      const double observed_ground = it->second.z() + message->point.z; // 同一扫描的 body 高度加相对地面 Z，得到 odom 地面高度（m）
      // 首次实测直接建基准；之后过大跳变保留旧高度并触发诊断
      ground_jump_rejected_ = !ground_height_.update(observed_ground, max_ground_innovation_);
      break;
    }
  }
}

// 墙钟检测位姿是否中断，再补充重力和地面参考状态
void LioNavBridge::publish_health()
{
  diagnostic_msgs::msg::DiagnosticArray array; // 桥接节点当前健康诊断消息
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status; // 记录重力初始化、位姿超时和地面高度状态
  status.name = "lidar_navigation.pose";
  status.hardware_id = "FAST_LIO";
  const double age = std::chrono::duration<double>(std::chrono::steady_clock::now()-last_odometry_).count(); // 上次有效位姿到达后经过的墙钟时长（s）
  status.level = odometry_received_ && age <= input_timeout_ ? 0 : 2;
  // 区分等待初始化、有效输入和里程计过期
  status.message = !initialized_ ? "WAITING_FOR_STATIC_IMU" :
    (status.level == 0 ? "OK" : "ODOMETRY_STALE");
  // 即使位姿正常，地面尚未建立或跳变被拒绝也要报告警告
  if (status.level == 0 && (!ground_height_.initialized() || ground_jump_rejected_))
  {
    status.level = 1;
    status.message = !ground_height_.initialized() ? "WAITING_FOR_GROUND" : "GROUND_REFERENCE_JUMP";
  }
  array.status.push_back(status);
  health_pub_->publish(array);
}
}
