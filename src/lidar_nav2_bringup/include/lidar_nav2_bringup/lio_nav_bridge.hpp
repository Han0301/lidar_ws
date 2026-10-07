#pragma once
#include "lidar_nav2_bringup/ground_height_filter.hpp"
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace lidar_nav2_bringup
{
class LioNavBridge : public rclcpp::Node
{
public:
  LioNavBridge();
private:
  void initialize_reference(sensor_msgs::msg::Imu::ConstSharedPtr message);
  void project_pose(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void ground_reference(geometry_msgs::msg::PointStamped::ConstSharedPtr message);
  void publish_health();
  Eigen::Vector3d acceleration_sum_ = Eigen::Vector3d::Zero();
  int initialization_samples_ = 0;
  int required_samples_ = 100;
  bool initialized_ = false;
  bool odometry_received_ = false;
  Eigen::Matrix3d level_from_camera_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d sensor_translation_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d previous_position_ = Eigen::Vector3d::Zero();
  double previous_yaw_ = 0.0;
  double previous_stamp_ = 0.0;
  GroundHeightFilter ground_height_;
  bool ground_jump_rejected_ = false;
  double max_ground_innovation_ = 0.5;
  double input_timeout_ = 0.6;
  std::chrono::steady_clock::time_point last_odometry_;
  std::deque<std::pair<double, Eigen::Vector3d>> pose_history_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ground_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};
}
