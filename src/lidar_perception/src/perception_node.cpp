#include "lidar_perception/perception_node.hpp"

// 雷达障碍物感知节点
namespace lidar_perception
{
namespace
{
// 将 TF 平移与四元数转换为 Eigen 仿射变换，方便逐点坐标计算
Eigen::Affine3f affine(const geometry_msgs::msg::Transform & transform)
{
  const auto & q = transform.rotation;      // TF 消息中的旋转四元数，分量顺序为 x/y/z/w
  Eigen::Affine3f matrix = Eigen::Affine3f::Identity();      // 从消息源坐标系到目标坐标系的刚体变换
  matrix.linear() = Eigen::Quaternionf(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
  matrix.translation() = Eigen::Vector3f(transform.translation.x,
    transform.translation.y, transform.translation.z);
  return matrix;
}
}

// 声明参数并建立流水线、TF、订阅发布接口和墙钟定时器
PerceptionNode::PerceptionNode() : Node("lidar_perception")
{
  // 读取 YAML 配置；几何阈值的单位和用途见 Parameters 定义
  const auto topic = declare_parameter<std::string>("input_topic", "/cloud_registered_body");      // FAST-LIO2 去畸变点云订阅话题
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
  // 启动时校验距离、地面方法和聚类阈值，拒绝无法运行的配置
  if (parameters_.min_range <= 0.0 || parameters_.max_range <= parameters_.min_range ||
    parameters_.voxel_size <= 0.0 || parameters_.ground_distance <= 0.0 ||
    parameters_.ground_guard_distance < parameters_.ground_distance ||
    parameters_.cluster_tolerance <= 0.0 || parameters_.cluster_min_points < 1 ||
    parameters_.obstacle_max_height <= parameters_.obstacle_min_height ||
    (parameters_.ground_method != "patchwork" && parameters_.ground_method != "plane"))
  {
    throw std::invalid_argument("Invalid perception parameters");
  }
  // 仅在用户明确配置空间边界时启用固定盒；空配置不删除任何区域
  const auto self = declare_parameter<std::vector<double>>("self_box_body", std::vector<double>());      // 可选固定盒边界：xmin/xmax/ymin/ymax/zmin/zmax，单位 m
  if (!self.empty())
  {
    if (self.size() != 6 || self[1] <= self[0] || self[3] <= self[2] || self[5] <= self[4])
    {
      throw std::invalid_argument("self_box_body must contain xmin,xmax,ymin,ymax,zmin,zmax");
    }
    ExclusionBox box;      // 统一表示 body→盒局部坐标的排除区域
    box.half_size = Eigen::Vector3f((self[1]-self[0])/2, (self[3]-self[2])/2, (self[5]-self[4])/2);
    box.box_from_body.translation() = -Eigen::Vector3f((self[1]+self[0])/2, (self[3]+self[2])/2, (self[5]+self[4])/2);
    box.id = "configured_holder_region";
    fixed_boxes_.push_back(box);
  }
  // 建立处理核心与 TF 缓存，点云调平依赖桥接节点提供的坐标关系
  pipeline_ = std::make_unique<CloudPipeline>(parameters_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(3);      // Best Effort、最多三帧缓存，优先处理近期点云
  // 建立点云、几何框、地面参考和诊断输出
  ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/ground", sensor_qos);
  obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/obstacles", sensor_qos);
  clearing_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/clearing", sensor_qos);
  removed_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/perception/operator_removed", sensor_qos);
  objects_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("/perception/objects", 3);
  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/perception/markers", 3);
  ground_reference_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/perception/ground_reference", 3);
  diagnostic_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/perception/diagnostics", 10);
  // 订阅去畸变点云和可选区域标签；原始雷达仍直接进入 FAST-LIO2
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(topic, sensor_qos,
    std::bind(&PerceptionNode::receive_cloud, this, std::placeholders::_1));
  regions_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>("/perception/operator_regions", 3,
    std::bind(&PerceptionNode::receive_regions, this, std::placeholders::_1));
  // 用墙钟定时器重试待处理帧，并在回放暂停时仍检查输入健康
  processing_timer_ = create_wall_timer(std::chrono::milliseconds(10),
    std::bind(&PerceptionNode::process_pending, this));
  health_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]()
    {
      const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now()-last_received_).count();      // 距离上次点云到达的墙钟时长（s）
      if (!received_ || age > input_timeout_)
      {
        publish_diagnostic(nullptr, received_ ? "INPUT_STALE" : "NO_DATA", 2);
      }
    });
}

// 记录输入到达时间、限制队列长度，并尝试处理队首帧
void PerceptionNode::receive_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  received_ = true;
  last_received_ = std::chrono::steady_clock::now();
  pending_.push_back(message);
  // 只保留三帧，避免缺 TF 时积压点云并不断增加延迟
  while (pending_.size() > 3)
  {
    pending_.pop_front();
    ++dropped_;
  }
  process_pending();
}

// 缓存外部区域标签，时效和坐标可用性在实际处理点云时再判断
void PerceptionNode::receive_regions(visualization_msgs::msg::MarkerArray::ConstSharedPtr message)
{
  regions_ = *message;
}

// 为队首帧查询同一时间戳的 TF、整理区域并执行一次感知处理
void PerceptionNode::process_pending()
{
  if (pending_.empty())
  {
    return;
  }
  const auto message = pending_.front();      // 等待队列中最旧的点云，时间戳用于 TF 与区域匹配
  Eigen::Affine3f level_from_body;      // 将输入帧坐标变到 output_frame_ 的同一扫描时刻变换
  // 查询失败时等待或放弃该帧或区域，不在错误坐标中删除点
  try
  {
    level_from_body = affine(tf_buffer_->lookupTransform(output_frame_, message->header.frame_id,
      rclcpp::Time(message->header.stamp)).transform);
  }
  catch (const tf2::TransformException & exception)
  {
    // TF 等待超过 0.5 s 仿真/ROS 时间时丢弃该帧，避免队首持续阻塞
    if ((now()-rclcpp::Time(message->header.stamp)).seconds() > 0.5)
    {
      pending_.pop_front();
      ++dropped_;
      publish_diagnostic(nullptr, "TF_UNAVAILABLE", 2);
    }
    return;
  }
  std::vector<ExclusionBox> boxes = fixed_boxes_;      // 本帧排除区域集合，先复制固定盒，再追加有效动态盒
  // 只接纳有效且与点云时间相近的区域，过期区域不参与删除
  for (const auto & region : regions_.markers)
  {
    if (region.action != visualization_msgs::msg::Marker::ADD || region.scale.x <= 0.0 ||
      region.scale.y <= 0.0 || region.scale.z <= 0.0 ||
      std::abs((rclcpp::Time(message->header.stamp)-rclcpp::Time(region.header.stamp)).seconds()) > region_timeout_)
    {
      continue;
    }
    // 查询失败时等待或放弃该帧或区域，不在错误坐标中删除点
    try
    {
      Eigen::Affine3f frame_from_box = Eigen::Affine3f::Identity();      // Marker 盒局部坐标到其 header.frame_id 的位姿
      const auto & q = region.pose.orientation;      // 区域盒的旋转四元数
      const Eigen::Quaternionf rotation(q.w, q.x, q.y, q.z);      // Eigen 构造顺序为 w/x/y/z，后面检查并归一化
      if (rotation.norm() < 0.1)
      {
        continue;
      }
      frame_from_box.linear() = rotation.normalized().toRotationMatrix();
      frame_from_box.translation() = Eigen::Vector3f(region.pose.position.x,
        region.pose.position.y, region.pose.position.z);
      const auto body_from_frame = affine(tf_buffer_->lookupTransform(message->header.frame_id,
        region.header.frame_id, rclcpp::Time(message->header.stamp)).transform);      // 区域消息坐标→点云 body 坐标
      ExclusionBox box;      // 统一表示 body→盒局部坐标的排除区域
      // 先得到盒→body，再求逆成 body→盒，供包含关系检查
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
  // 将 ROS 消息转为 PCL，调用不依赖 ROS 的处理核心
  Cloud cloud;      // 从 PointCloud2 转换得到的原始输入坐标点云
  pcl::fromROSMsg(*message, cloud);
  const auto result = pipeline_->process(cloud, level_from_body, boxes);      // 本帧感知结果，含可信地面标志和诊断统计
  pending_.pop_front();
  ++processed_;
  auto header = message->header;      // 保留扫描时间，仅改成处理结果的输出坐标系
  header.frame_id = output_frame_;
  publish_result(result, header);
  // 地面可信时报告 OK；不可信时发布警告，同时门控地图点云
  publish_diagnostic(&result, result.ground_reference_valid ? "OK" : "GROUND_UNCERTAIN",
    result.ground_reference_valid ? 0 : 1);
}

// 统一转换并发布点云，输出继承扫描时间戳和指定坐标系
void PerceptionNode::publish_cloud
(
  const Cloud & cloud,
  const std_msgs::msg::Header & header,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher
)
{
  sensor_msgs::msg::PointCloud2 message;      // 待发布的 ROS 点云消息
  pcl::toROSMsg(cloud, message);
  message.header = header;
  publisher->publish(message);
}

// 发布分类点云和几何框，并在可信地面条件下反馈高度参考
void PerceptionNode::publish_result(const Result & result, const std_msgs::msg::Header & header)
{
  // 地面点供观察；只有本帧地面可信才发布障碍标记和清除输入
  publish_cloud(*result.ground, header, ground_pub_);
  // 地图输入和高度反馈都依赖当前帧的可信地面观测
  if (result.ground_reference_valid)
  {
    publish_cloud(*result.obstacles, header, obstacle_pub_);
    publish_cloud(*result.clearing, header, clearing_pub_);
  }
  publish_cloud(*result.removed, header, removed_pub_);
  // 构造当前帧几何框；每帧 ID 重置，不能当作跨帧跟踪身份
  vision_msgs::msg::Detection3DArray detections;      // 本帧几何包围框集合，不填语义分类结果
  detections.header = header;
  visualization_msgs::msg::MarkerArray markers;      // 先清理旧框，再绘制本帧几何框的 RViz 消息集合
  visualization_msgs::msg::Marker clear;      // DELETEALL 指令，避免旧帧更多的框残留
  clear.header = header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);
  int index = 0;      // 本帧框编号，每帧重新从零开始，不代表跟踪 ID
  for (const auto & object : result.objects)
  {
    vision_msgs::msg::Detection3D detection;      // 一个轴对齐几何包围框及当前帧编号
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
    visualization_msgs::msg::Marker marker;      // 该几何框对应的半透明红色 RViz 立方体
    marker.header = header;
    marker.ns = "obstacle_clusters";
    marker.id = index++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = detection.bbox.center;
    // 给可视化边长设置最小值，避免单轴过薄的框在 RViz 中不可见
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
  // 地图输入和高度反馈都依赖当前帧的可信地面观测
  if (result.ground_reference_valid)
  {
    geometry_msgs::msg::PointStamped reference;      // 输出坐标原点正下方的可信参考地面位置
    reference.header = header;
    reference.point.z = -result.sensor_height;      // 原点位于地面上方，所以相对地面 Z 为负高度（m）
    ground_reference_pub_->publish(reference);
  }
}

// 将累计帧数和可用的单帧统计封装成标准 ROS 诊断消息
void PerceptionNode::publish_diagnostic
(
  const Result * result,
  const std::string & state,
  int level
)
{
  diagnostic_msgs::msg::DiagnosticArray array;      // 当前时刻的诊断消息集合
  array.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus status;      // 流水线诊断项：等级、状态文本和统计键值
  status.name = "lidar_perception.pipeline";
  status.hardware_id = "MID360";
  status.level = level;
  status.message = state;
  // 小型辅助函数：向同一诊断项追加一对键值
  const auto add = [&status](const std::string & key, const std::string & value)
    {
      diagnostic_msgs::msg::KeyValue entry;      // 一项可供终端或诊断工具读取的统计值
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    };
  add("processed", std::to_string(processed_));
  add("dropped", std::to_string(dropped_));
  // 只有完成了流水线处理才附带单帧点数、高度和耗时
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
