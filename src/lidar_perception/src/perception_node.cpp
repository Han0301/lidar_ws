#include "lidar_perception/perception_node.hpp"

namespace lidar_perception
{
namespace
{
Eigen::Affine3f affine(const geometry_msgs::msg::Transform & transform)
{
  const auto & q = transform.rotation;
  Eigen::Affine3f matrix = Eigen::Affine3f::Identity();
  matrix.linear() = Eigen::Quaternionf(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  matrix.translation() = Eigen::Vector3f(transform.translation.x,
    transform.translation.y, transform.translation.z);
  return matrix;
}
}

PerceptionNode::PerceptionNode()
: Node("lidar_perception")
{
  const auto topic = declare_parameter<std::string>("input_topic", "/cloud_registered_body");
  output_frame_ = declare_parameter<std::string>("output_frame", "perception_sensor");
  region_timeout_ = declare_parameter<double>("operator_region_timeout_s", 0.3);
  input_timeout_ = declare_parameter<double>("input_timeout_s", 0.6);
  parameters_.min_range = declare_parameter<double>("min_range", parameters_.min_range);
  parameters_.max_range = declare_parameter<double>("max_range", parameters_.max_range);
  parameters_.voxel_size = declare_parameter<double>("voxel_size", parameters_.voxel_size);
  parameters_.ground_distance = declare_parameter<double>("ground_distance", parameters_.ground_distance);
  parameters_.ground_guard_distance = declare_parameter<double>("ground_guard_distance", parameters_.ground_guard_distance);
  parameters_.obstacle_min_height = declare_parameter<double>("obstacle_min_height", parameters_.obstacle_min_height);
  parameters_.obstacle_max_height = declare_parameter<double>("obstacle_max_height", parameters_.obstacle_max_height);
  parameters_.cluster_tolerance = declare_parameter<double>("cluster_tolerance", parameters_.cluster_tolerance);
  parameters_.cluster_min_points = declare_parameter<int>("cluster_min_points", parameters_.cluster_min_points);
  parameters_.cluster_max_points = declare_parameter<int>("cluster_max_points", parameters_.cluster_max_points);
  parameters_.ground_method = declare_parameter<std::string>("ground_method", parameters_.ground_method);
  if (parameters_.min_range <= 0.0 || parameters_.max_range <= parameters_.min_range ||
    parameters_.voxel_size <= 0.0 || parameters_.ground_distance <= 0.0 ||
    parameters_.ground_guard_distance < parameters_.ground_distance ||
    parameters_.cluster_tolerance <= 0.0 || parameters_.cluster_min_points < 1 ||
    parameters_.obstacle_max_height <= parameters_.obstacle_min_height ||
    (parameters_.ground_method != "patchwork" && parameters_.ground_method != "plane"))
  {
    throw std::invalid_argument("Invalid perception parameters");
  }
  const auto self = declare_parameter<std::vector<double>>("self_box_body", std::vector<double>());
  if (!self.empty())
  {
    if (self.size() != 6 || self[1] <= self[0] || self[3] <= self[2] || self[5] <= self[4])
    {
      throw std::invalid_argument("self_box_body must contain xmin,xmax,ymin,ymax,zmin,zmax");
    }
    ExclusionBox box;
    box.half_size = Eigen::Vector3f((self[1]-self[0])/2, (self[3]-self[2])/2, (self[5]-self[4])/2);
    box.box_from_body.translation() = -Eigen::Vector3f((self[1]+self[0])/2, (self[3]+self[2])/2, (self[5]+self[4])/2);
    box.id = "configured_holder_region";
    fixed_boxes_.push_back(box);
  }
  pipeline_ = std::make_unique<CloudPipeline>(parameters_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(3);
  ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground", sensor_qos);
  obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/obstacles", sensor_qos);
  clearing_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/clearing", sensor_qos);
  removed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/operator_removed", sensor_qos);
  objects_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("/perception/objects", 3);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/perception/markers", 3);
  ground_reference_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/perception/ground_reference", 3);
  diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/perception/diagnostics", 10);
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic, sensor_qos,
    std::bind(&PerceptionNode::receive_cloud, this, std::placeholders::_1));
  regions_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>("/perception/operator_regions", 3,
    std::bind(&PerceptionNode::receive_regions, this, std::placeholders::_1));
  processing_timer_ = create_wall_timer(std::chrono::milliseconds(10),
    std::bind(&PerceptionNode::process_pending, this));
  health_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]()
    {
      const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now()-last_received_).count();
      if (!received_ || age > input_timeout_)
      {
        publish_diagnostic(nullptr, received_ ? "INPUT_STALE" : "NO_DATA", 2);
      }
    });
}

void PerceptionNode::receive_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  received_ = true;
  last_received_ = std::chrono::steady_clock::now();
  pending_.push_back(message);
  while (pending_.size() > 3)
  {
    pending_.pop_front();
    ++dropped_;
  }
  process_pending();
}

void PerceptionNode::receive_regions(visualization_msgs::msg::MarkerArray::ConstSharedPtr message)
{
  regions_ = *message;
}

void PerceptionNode::process_pending()
{
  if (pending_.empty())
  {
    return;
  }
  const auto message = pending_.front();
  Eigen::Affine3f level_from_body;
  try
  {
    level_from_body = affine(tf_buffer_->lookupTransform(output_frame_, message->header.frame_id,
      rclcpp::Time(message->header.stamp)).transform);
  }
  catch (const tf2::TransformException & exception)
  {
    if ((now()-rclcpp::Time(message->header.stamp)).seconds() > 0.5)
    {
      pending_.pop_front();
      ++dropped_;
      publish_diagnostic(nullptr, "TF_UNAVAILABLE", 2);
    }
    return;
  }
  std::vector<ExclusionBox> boxes = fixed_boxes_;
  for (const auto & region : regions_.markers)
  {
    if (region.action != visualization_msgs::msg::Marker::ADD || region.scale.x <= 0.0 ||
      region.scale.y <= 0.0 || region.scale.z <= 0.0 ||
      std::abs((rclcpp::Time(message->header.stamp)-rclcpp::Time(region.header.stamp)).seconds()) > region_timeout_)
    {
      continue;
    }
    try
    {
      Eigen::Affine3f frame_from_box = Eigen::Affine3f::Identity();
      const auto & q = region.pose.orientation;
      const Eigen::Quaternionf rotation(q.w, q.x, q.y, q.z);
      if (rotation.norm() < 0.1)
      {
        continue;
      }
      frame_from_box.linear() = rotation.normalized().toRotationMatrix();
      frame_from_box.translation() = Eigen::Vector3f(region.pose.position.x,
        region.pose.position.y, region.pose.position.z);
      const auto body_from_frame = affine(tf_buffer_->lookupTransform(message->header.frame_id,
        region.header.frame_id, rclcpp::Time(message->header.stamp)).transform);
      ExclusionBox box;
      box.box_from_body = (body_from_frame * frame_from_box).inverse();
      box.half_size = Eigen::Vector3f(region.scale.x/2, region.scale.y/2, region.scale.z/2);
      box.id = region.ns;
      boxes.push_back(box);
    }
    catch (const tf2::TransformException & exception)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Operator region TF missing; retaining obstacle");
    }
  }
  Cloud cloud;
  pcl::fromROSMsg(*message, cloud);
  const auto result = pipeline_->process(cloud, level_from_body, boxes);
  pending_.pop_front();
  ++processed_;
  auto header = message->header;
  header.frame_id = output_frame_;
  publish_result(result, header);
  publish_diagnostic(&result, result.ground_reference_valid ? "OK" : "GROUND_UNCERTAIN",
    result.ground_reference_valid ? 0 : 1);
}

void PerceptionNode::publish_cloud(const Cloud & cloud, const std_msgs::msg::Header & header,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud, message);
  message.header = header;
  publisher->publish(message);
}

void PerceptionNode::publish_result(const Result & result, const std_msgs::msg::Header & header)
{
  publish_cloud(*result.ground, header, ground_pub_);
  if (result.ground_reference_valid)
  {
    publish_cloud(*result.obstacles, header, obstacle_pub_);
    publish_cloud(*result.clearing, header, clearing_pub_);
  }
  publish_cloud(*result.removed, header, removed_pub_);
  vision_msgs::msg::Detection3DArray detections;
  detections.header = header;
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker clear;
  clear.header = header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  int index = 0;
  for (const auto & object : result.objects)
  {
    vision_msgs::msg::Detection3D detection;
    detection.header = header;
    detection.id = std::to_string(index);
    detection.bbox.center.position.x = object.center.x();
    detection.bbox.center.position.y = object.center.y();
    detection.bbox.center.position.z = object.center.z();
    detection.bbox.center.orientation.w = 1.0;
    detection.bbox.size.x = object.size.x();
    detection.bbox.size.y = object.size.y();
    detection.bbox.size.z = object.size.z();
    detections.detections.push_back(detection);
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "obstacle_clusters";
    marker.id = index++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = detection.bbox.center;
    marker.scale.x = std::max(0.05, detection.bbox.size.x);
    marker.scale.y = std::max(0.05, detection.bbox.size.y);
    marker.scale.z = std::max(0.05, detection.bbox.size.z);
    marker.color.r = 1.0F;
    marker.color.g = 0.3F;
    marker.color.a = 0.35F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    markers.markers.push_back(marker);
  }
  objects_pub_->publish(detections);
  markers_pub_->publish(markers);
  if (result.ground_reference_valid)
  {
    geometry_msgs::msg::PointStamped reference;
    reference.header = header;
    reference.point.z = -result.sensor_height;
    ground_reference_pub_->publish(reference);
  }
}

void PerceptionNode::publish_diagnostic(const Result * result, const std::string & state, int level)
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "lidar_perception.pipeline";
  status.hardware_id = "MID360";
  status.level = level;
  status.message = state;
  const auto add = [&status](const std::string & key, const std::string & value)
    {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    };
  add("processed", std::to_string(processed_));
  add("dropped", std::to_string(dropped_));
  if (result)
  {
    add("input_points", std::to_string(result->input_points));
    add("valid_points", std::to_string(result->valid_points));
    add("operator_points", std::to_string(result->operator_points));
    add("ground_points", std::to_string(result->ground->size()));
    add("obstacle_points", std::to_string(result->obstacles->size()));
    add("clusters", std::to_string(result->objects.size()));
    add("sensor_height_m", std::to_string(result->sensor_height));
    add("processing_ms", std::to_string(result->processing_ms));
  }
  array.status.push_back(status);
  diagnostic_pub_->publish(array);
}
}
