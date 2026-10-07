#include "lidar_nav2_bringup/lio_nav_bridge.hpp"

namespace lidar_nav2_bringup
{
namespace
{
geometry_msgs::msg::TransformStamped make_transform(const std::string & parent, const std::string & child,
  const builtin_interfaces::msg::Time & stamp, const Eigen::Vector3d & position,
  const Eigen::Quaterniond & rotation)
{
  geometry_msgs::msg::TransformStamped output;
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

LioNavBridge::LioNavBridge()
: Node("lio_nav_bridge")
{
  required_samples_ = declare_parameter<int>("gravity_samples", 100);
  input_timeout_ = declare_parameter<double>("input_timeout_s", 0.6);
  max_ground_innovation_ = declare_parameter<double>("max_ground_innovation_m", 0.5);
  const auto translation = declare_parameter<std::vector<double>>("lidar_translation_in_body",
    std::vector<double>(3, 0.0));
  if (translation.size() != 3 || required_samples_ < 10 || max_ground_innovation_ <= 0.0)
  {
    throw std::invalid_argument("Invalid navigation frame initialization parameters");
  }
  sensor_translation_ = Eigen::Vector3d(translation[0], translation[1], translation[2]);
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

void LioNavBridge::initialize_reference(sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  if (initialized_)
  {
    return;
  }
  const Eigen::Vector3d acceleration(message->linear_acceleration.x,
    message->linear_acceleration.y, message->linear_acceleration.z);
  const Eigen::Vector3d angular(message->angular_velocity.x,
    message->angular_velocity.y, message->angular_velocity.z);
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
  level_from_camera_ = Eigen::Quaterniond::FromTwoVectors(
    acceleration_sum_.normalized(), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.push_back(make_transform("map", "odom", message->header.stamp,
    Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()));
  transforms.push_back(make_transform("odom", "camera_init", message->header.stamp,
    Eigen::Vector3d::Zero(), Eigen::Quaterniond(level_from_camera_)));
  static_broadcaster_->sendTransform(transforms);
  initialized_ = true;
  RCLCPP_INFO(get_logger(), "Gravity reference initialized from %d stationary IMU samples", initialization_samples_);
}

void LioNavBridge::project_pose(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  if (!initialized_ || message->header.frame_id != "camera_init" || message->child_frame_id != "body")
  {
    return;
  }
  const auto & p = message->pose.pose.position;
  const auto & q = message->pose.pose.orientation;
  const Eigen::Quaterniond camera_from_body(q.w, q.x, q.y, q.z);
  if (camera_from_body.norm() < 0.5)
  {
    return;
  }
  const Eigen::Matrix3d world_from_body = level_from_camera_ * camera_from_body.normalized().toRotationMatrix();
  const Eigen::Vector3d position = level_from_camera_ * Eigen::Vector3d(p.x, p.y, p.z);
  const double yaw = std::atan2(world_from_body(1, 0), world_from_body(0, 0));
  const Eigen::Quaterniond yaw_rotation(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d base_position = position;
  base_position.z() = ground_height_.height();
  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  transforms.push_back(make_transform("odom", "nav_base", message->header.stamp, base_position, yaw_rotation));
  transforms.push_back(make_transform("odom", "perception_sensor", message->header.stamp, position, yaw_rotation));
  transforms.push_back(make_transform("odom", "nav_sensor", message->header.stamp,
    position + world_from_body * sensor_translation_, Eigen::Quaterniond(world_from_body)));
  broadcaster_->sendTransform(transforms);
  nav_msgs::msg::Odometry output;
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
  Eigen::Matrix<double, 6, 6> covariance;
  for (int row = 0; row < 6; ++row)
  {
    for (int col = 0; col < 6; ++col)
    {
      covariance(row, col) = message->pose.covariance[row * 6 + col];
    }
  }
  Eigen::Matrix<double, 6, 6> jacobian = Eigen::Matrix<double, 6, 6>::Zero();
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
  const double stamp = rclcpp::Time(message->header.stamp).seconds();
  const double duration = stamp - previous_stamp_;
  if (odometry_received_ && duration > 0.02 && duration < 0.30)
  {
    const Eigen::Vector3d velocity = yaw_rotation.inverse() * ((position-previous_position_)/duration);
    output.twist.twist.linear.x = velocity.x();
    output.twist.twist.linear.y = velocity.y();
    output.twist.twist.angular.z = std::remainder(yaw-previous_yaw_, 2*M_PI)/duration;
  }
  odometry_pub_->publish(output);
  previous_position_ = position;
  previous_yaw_ = yaw;
  previous_stamp_ = stamp;
  pose_history_.emplace_back(stamp, position);
  while (pose_history_.size() > 20)
  {
    pose_history_.pop_front();
  }
  odometry_received_ = true;
  last_odometry_ = std::chrono::steady_clock::now();
}

void LioNavBridge::ground_reference(geometry_msgs::msg::PointStamped::ConstSharedPtr message)
{
  if (message->header.frame_id != "perception_sensor" || !std::isfinite(message->point.z))
  {
    return;
  }
  const double stamp = rclcpp::Time(message->header.stamp).seconds();
  for (auto it = pose_history_.rbegin(); it != pose_history_.rend(); ++it)
  {
    if (std::abs(it->first - stamp) < 0.02)
    {
      const double observed_ground = it->second.z() + message->point.z;
      ground_jump_rejected_ = !ground_height_.update(observed_ground, max_ground_innovation_);
      break;
    }
  }
}

void LioNavBridge::publish_health()
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "lidar_navigation.pose";
  status.hardware_id = "FAST_LIO";
  const double age = std::chrono::duration<double>(std::chrono::steady_clock::now()-last_odometry_).count();
  status.level = odometry_received_ && age <= input_timeout_ ? 0 : 2;
  status.message = !initialized_ ? "WAITING_FOR_STATIC_IMU" :
    (status.level == 0 ? "OK" : "ODOMETRY_STALE");
  if (status.level == 0 && (!ground_height_.initialized() || ground_jump_rejected_))
  {
    status.level = 1;
    status.message = !ground_height_.initialized() ? "WAITING_FOR_GROUND" : "GROUND_REFERENCE_JUMP";
  }
  array.status.push_back(status);
  health_pub_->publish(array);
}
}
