#pragma once
#include "lidar_perception/cloud_pipeline.hpp"
#include <deque>
#include <functional>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

namespace lidar_perception
{
// 负责 ROS 参数、点云/区域接收、时间戳 TF 查询、发布与健康诊断
class PerceptionNode : public rclcpp::Node
{
public:
  // 建立参数、流水线、订阅发布接口和墙钟定时器
  PerceptionNode();
private:
  // 缓存输入并触发处理，队列满时丢弃最旧帧
  void receive_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  // 缓存动态排除区域；有效性在处理对应点云时检查
  void receive_regions(visualization_msgs::msg::MarkerArray::ConstSharedPtr message);
  // 获取队首点云对应时刻的 TF，并调用感知流水线
  void process_pending();
  // 发布点云、几何框和地面参考；地面不可信时暂停地图输入
  void publish_result(const Result & result, const std_msgs::msg::Header & header);
  // 发布诊断等级、原因和可用的单帧统计，result 可以为空
  void publish_diagnostic
  (
    const Result * result,
    const std::string & state,
    int level
  );
  // 统一将 PCL 点云转换为 PointCloud2，并保留帧时间戳
  void publish_cloud
  (
    const Cloud & cloud,
    const std_msgs::msg::Header & header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher
  );
  Parameters parameters_;      // 从 YAML 读取的几何处理参数
  std::unique_ptr<CloudPipeline> pipeline_;      // 不依赖 ROS 消息格式的单帧处理流水线
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;      // 缓存按消息时间戳查询的坐标变换
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;      // 订阅 TF 并更新缓存
  std::string output_frame_;      // 调平点云的输出坐标系，默认 perception_sensor
  double region_timeout_ = 0.3;      // 排除区域与当前点云允许的时间戳差（s）
  double input_timeout_ = 0.6;      // 墙钟接收超时阈值（s），回放暂停也能检测
  std::vector<ExclusionBox> fixed_boxes_;      // 在 body 坐标系配置的固定排除区域
  visualization_msgs::msg::MarkerArray regions_;      // 最近收到的动态区域描述，使用时再检查时效和 TF
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_;      // 等待对应 TF 的点云队列，最多保留三帧
  std::chrono::steady_clock::time_point last_received_;      // 上次收到点云的单调墙钟时间
  bool received_ = false;      // 是否曾收到输入，用于区分无数据和输入过期
  std::size_t processed_ = 0;      // 已经完成流水线处理的帧数
  std::size_t dropped_ = 0;      // 因队列溢出或 TF 等待超时而丢弃的帧数

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;      // FAST-LIO2 去畸变 body 点云输入
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr regions_sub_;      // 外部提供的空间排除区域输入
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;      // 地面点云输出
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;      // 可信地面条件下的障碍标记输入
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_pub_;      // 可信地面条件下的真实射线终点
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr removed_pub_;      // 排除区域内点的诊断输出
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr objects_pub_;      // 当前帧几何包围框输出
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;      // 供 RViz 显示的聚类框
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ground_reference_pub_;      // 反馈给桥接节点的本帧可信地面高度
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;      // 输入状态、处理耗时和点数统计
  rclcpp::TimerBase::SharedPtr processing_timer_;      // 每 10 ms 重试等待 TF 的队首点云
  rclcpp::TimerBase::SharedPtr health_timer_;      // 每 500 ms 检查墙钟输入超时
};
}
