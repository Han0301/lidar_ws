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
// 保留 LIO 位姿，建立重力对齐坐标和手持数据的虚拟平面导航参考
class LioNavBridge : public rclcpp::Node
{
public:
  // 声明外参、初始化与超时参数，并建立桥接话题和 TF 接口
  LioNavBridge();
private:
  // 累计低角速度样本，仅在启动时估计固定重力方向
  void initialize_reference(sensor_msgs::msg::Imu::ConstSharedPtr message);
  // 将 LIO 位姿转到 odom，发布平面参考、真实传感器 TF 和差分速度
  void project_pose(nav_msgs::msg::Odometry::ConstSharedPtr message);
  // 按扫描时间匹配位置，将相对地面高度转换为 odom 地面高度
  void ground_reference(geometry_msgs::msg::PointStamped::ConstSharedPtr message);
  // 以墙钟检测位姿中断，并报告重力/地面参考状态
  void publish_health();
  Eigen::Vector3d acceleration_sum_ = Eigen::Vector3d::Zero();      // 初始化期间低角速度 IMU 加速度的累计值
  int initialization_samples_ = 0;      // 已接受的初始化 IMU 样本数
  int required_samples_ = 100;      // 建立重力参考需要的样本数
  bool initialized_ = false;      // 重力参考和静态 TF 是否已经建立
  bool odometry_received_ = false;      // 是否已接受有效 LIO 位姿，也用于判断能否差分速度
  Eigen::Matrix3d level_from_camera_ = Eigen::Matrix3d::Identity();      // 从 camera_init 变到重力对齐 odom 的固定旋转
  Eigen::Vector3d sensor_translation_ = Eigen::Vector3d::Zero();      // LiDAR 原点在 body 中的平移外参（m），用于真实射线起点
  Eigen::Vector3d previous_position_ = Eigen::Vector3d::Zero();      // 上一帧 body 原点在 odom 中的位置（m）
  double previous_yaw_ = 0.0;      // 上一帧重力对齐后的航向角（rad）
  double previous_stamp_ = 0.0;      // 上一帧 LIO 消息时间（s）
  GroundHeightFilter ground_height_;      // 滤波后的 odom 地面高度，用于虚拟 nav_base 的 Z
  bool ground_jump_rejected_ = false;      // 最近一次匹配的地面高度是否因跳变被拒绝
  double max_ground_innovation_ = 0.5;      // 后续地面高度更新允许的最大差值（m）
  double input_timeout_ = 0.6;      // LIO 位姿墙钟接收超时阈值（s）
  std::chrono::steady_clock::time_point last_odometry_;      // 最近一次有效位姿到达的单调墙钟时间
  std::deque<std::pair<double, Eigen::Vector3d>> pose_history_;      // 最近二十帧的时间和 odom 位置，用于匹配地面观测
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;      // 发布 nav_base、perception_sensor 和 nav_sensor 动态 TF
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;      // 发布 map→odom 和 odom→camera_init 固定变换
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;      // 原始 IMU，仅用于一次重力初始化
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;      // FAST-LIO2 的 camera_init→body 位姿输入
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ground_sub_;      // 感知节点的可信地面参考输入
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;      // Nav2 可使用的 odom→nav_base 平面参考位姿
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr health_pub_;      // 初始化、位姿超时和地面高度跳变诊断
  rclcpp::TimerBase::SharedPtr health_timer_;      // 每 500 ms 检查导航输入健康状态
};
}
