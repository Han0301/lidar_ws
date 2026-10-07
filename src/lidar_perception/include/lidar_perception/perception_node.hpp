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
class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode();
private:
  void receive_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void receive_regions(visualization_msgs::msg::MarkerArray::ConstSharedPtr message);
  void process_pending();
  void publish_result(const Result & result, const std_msgs::msg::Header & header);
  void publish_diagnostic(const Result * result, const std::string & state, int level);
  void publish_cloud(const Cloud & cloud, const std_msgs::msg::Header & header,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher);
  Parameters parameters_;
  std::unique_ptr<CloudPipeline> pipeline_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string output_frame_;
  double region_timeout_ = 0.3;
  double input_timeout_ = 0.6;
  std::vector<ExclusionBox> fixed_boxes_;
  visualization_msgs::msg::MarkerArray regions_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending_;
  std::chrono::steady_clock::time_point last_received_;
  bool received_ = false;
  std::size_t processed_ = 0;
  std::size_t dropped_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr regions_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clearing_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr removed_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr objects_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ground_reference_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
};
}
