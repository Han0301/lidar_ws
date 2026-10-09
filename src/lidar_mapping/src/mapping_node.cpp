#include "lidar_mapping/occupancy_mapper.hpp"
#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <Eigen/Geometry>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace lidar_mapping
{

// 节点只处理同一扫描的分类消息和 TF，占用计算由 OctoMap 核心完成
class MappingNode : public rclcpp::Node
{
public:

  MappingNode() : Node("lidar_mapping")
  {
    Settings settings;
    settings.resolution = declare_parameter("resolution", 0.1);
    settings.max_range = declare_parameter("max_range", 15.0);
    settings.min_ground_observations = declare_parameter("min_ground_observations", 2);
    const int max_cells = declare_parameter("max_grid_cells", 4000000);
    if (max_cells <= 0)
    {
      throw std::invalid_argument("max_grid_cells must be positive");
    }
    settings.max_grid_cells = static_cast<std::size_t>(max_cells);
    settings.incremental_projection = declare_parameter("incremental_projection", false);
    settings.incremental_inner_updates = declare_parameter("incremental_inner_updates", false);
    settings.prune_interval = declare_parameter("prune_interval", 0);
    const int max_endpoints = declare_parameter("max_ray_endpoints", 0);
    if (max_endpoints < 0)
    {
      throw std::invalid_argument("max_ray_endpoints must be nonnegative");
    }
    settings.max_ray_endpoints = static_cast<std::size_t>(max_endpoints);
    mapper_ = std::make_unique<OccupancyMapper>(settings);
    resolution_ = settings.resolution;
    frame_ = declare_parameter<std::string>("global_frame", "odom");
    sensor_ = declare_parameter<std::string>("sensor_frame", "nav_sensor");
    output_ = declare_parameter<std::string>("output_prefix", "/tmp/lidar_global_map");
    const double integration_rate = declare_parameter("integration_rate", 2.0);
    const double publish_rate = declare_parameter("publish_rate", 1.0);
    if (integration_rate <= 0 || publish_rate <= 0)
    {
      throw std::invalid_argument("Mapping rates must be positive");
    }
    integration_interval_ns_ = static_cast<int64_t>(1e9 / integration_rate);
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    const int queue_depth = declare_parameter("pairing_queue_depth", 5);
    const int pending_frames = declare_parameter("max_pending_frames", 16);
    if (queue_depth < 1 || pending_frames < 1)
    {
      throw std::invalid_argument("Mapping pairing capacities must be positive");
    }
    max_pending_frames_ = static_cast<std::size_t>(pending_frames);
    const auto qos = rclcpp::SensorDataQoS().keep_last(queue_depth);
    const std::vector<std::string> topics =
    {
      "ground", "obstacles", "clearing"
    };
    for (const auto & topic : topics)
    {

      subscriptions_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(

        std::string("/perception/") + topic, qos,

        [this, label = std::string(topic)](sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
        {
          const int64_t stamp = rclcpp::Time(message->header.stamp).nanoseconds();
          ++received_[label];
          auto & pending = pending_[stamp];
          pending.messages[label] = message;
          while (pending_.size() > max_pending_frames_)
          {
            record_unpaired(pending_.begin()->second, pending_.begin()->first);
            pending_.erase(pending_.begin());
          }
        }));
    }
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).reliable().transient_local());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/mapping/diagnostics", 10);

    process_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]()
      {
        process_pending();
      });

    publish_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate), [this]()
      {
        publish_map();
      });

    save_service_ = create_service<std_srvs::srv::Trigger>("/mapping/save_map",

      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        response->success = save_map();
        response->message = response->success ? output_ + ".yaml" : "No complete map or save failed";
      });
    RCLCPP_INFO(get_logger(), "OctoMap session mapping in %s; output %s.yaml", frame_.c_str(), output_.c_str());
  }

  // 保存服务和正常退出共用同一非破坏性导出路径
  bool save_map() const
  {
    try
    {
      return mapper_->save(output_, frame_);
    }
    catch (const std::exception & error)
    {
      RCLCPP_ERROR(get_logger(), "Map save failed: %s", error.what());
      return false;
    }
  }

private:

  struct Pending
  {
    std::map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> messages;
    std::chrono::steady_clock::time_point arrived = std::chrono::steady_clock::now();
  };

  void record_unpaired(const Pending & pending, int64_t stamp)
  {
    ++unpaired_;
    std::string missing;
    const std::vector<std::string> labels =
    {
      "ground", "obstacles", "clearing"
    };
    for (const auto & label : labels)
    {
      if (pending.messages.count(label) == 0)
      {
        ++missing_[label];
        missing += missing.empty() ? label : std::string("+") + label;
      }
    }
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "lidar_mapping/pairing";
    status.message = "UNPAIRED";
    diagnostic_msgs::msg::KeyValue value;
    value.key = "scan_stamp_ns";
    value.value = std::to_string(stamp);
    status.values.push_back(value);
    value.key = "missing_topics";
    value.value = missing;
    status.values.push_back(value);
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  // 消息点变到固定世界系，TF 查询严格使用扫描时间
  std::vector<octomap::point3d> points(const sensor_msgs::msg::PointCloud2 & message, const geometry_msgs::msg::Transform & transform) const
  {
    const auto & q = transform.rotation;
    const auto rotation = Eigen::Quaternionf(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    const auto & t = transform.translation;
    const Eigen::Vector3f translation(t.x, t.y, t.z);
    std::vector<octomap::point3d> result;
    result.reserve(message.width * message.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(message, "z");
    for (; x != x.end(); ++x, ++y, ++z)
    {
      if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z))
      {
        const Eigen::Vector3f world = rotation * Eigen::Vector3f(*x, *y, *z) + translation;
        result.emplace_back(world.x(), world.y(), world.z());
      }
    }
    return result;
  }

  // 限制全局积分频率，局部感知不等待全局建图；过期帧与门控缺失分别计数
  void process_pending()
  {
    auto it = pending_.begin();
    while (it != pending_.end())
    {
      const auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - it->second.arrived).count();
      const auto stamp = it->first;
      const auto & messages = it->second.messages;
      if (messages.size() != 3)
      {
        if (age > 1.0)
        {
          record_unpaired(it->second, stamp);
          it = pending_.erase(it);
          continue;
        }
        ++it;
        continue;
      }
      if (stamp <= last_stamp_ || (last_stamp_ > 0 && stamp - last_stamp_ < integration_interval_ns_))
      {
        ++throttled_;
        it = pending_.erase(it);
        continue;
      }
      try
      {
        const auto & clearing = *messages.at("clearing");
        if (clearing.header.frame_id != messages.at("ground")->header.frame_id ||

          clearing.header.frame_id != messages.at("obstacles")->header.frame_id)
        {
          throw std::runtime_error("Classification frame mismatch");
        }
        const auto cloud_tf = buffer_->lookupTransform(frame_, clearing.header.frame_id, rclcpp::Time(stamp));
        const auto origin_tf = buffer_->lookupTransform(frame_, sensor_, rclcpp::Time(stamp));
        const auto began = std::chrono::steady_clock::now();
        const auto & p = origin_tf.transform.translation;
        const auto returns = points(clearing, cloud_tf.transform);
        const auto obstacles = points(*messages.at("obstacles"), cloud_tf.transform);
        const auto ground = points(*messages.at("ground"), cloud_tf.transform);
        const auto transform_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
        mapper_->integrate(octomap::point3d(p.x, p.y, p.z), returns, obstacles, ground);
        integration_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
        last_stamp_ = stamp;
        ++integrated_;
        diagnostic_msgs::msg::DiagnosticArray timing;
        timing.header.stamp = now();
        timing.header.frame_id = frame_;
        diagnostic_msgs::msg::DiagnosticStatus sample;
        sample.name = "lidar_mapping/integration";
        sample.message = "INTEGRATED";
        diagnostic_msgs::msg::KeyValue elapsed;
        elapsed.key = "integration_ms";
        elapsed.value = std::to_string(integration_ms_);
        sample.values.push_back(elapsed);
        elapsed.key = "scan_stamp_ns";
        elapsed.value = std::to_string(stamp);
        sample.values.push_back(elapsed);
        elapsed.key = "transform_ms";
        elapsed.value = std::to_string(transform_ms);
        sample.values.push_back(elapsed);
        for (const auto & stage : mapper_->timings())
        {
          elapsed.key = stage.first;
          elapsed.value = std::to_string(stage.second);
          sample.values.push_back(elapsed);
        }
        timing.status.push_back(sample);
        diagnostics_pub_->publish(timing);
        it = pending_.erase(it);
      }
      catch (const tf2::TransformException &)
      {
        if (age > 1.0)
        {
          ++tf_dropped_;
          it = pending_.erase(it);
        }
        else
        {
          ++it;
        }
      }
      catch (const std::exception & error)
      {
        RCLCPP_ERROR(get_logger(), "Mapping frame rejected: %s", error.what());
        ++rejected_;
        it = pending_.erase(it);
      }
    }
  }

  // 全局地图低频发布，避免整图序列化占用局部更新的预算
  void publish_map()
  {
    const auto began = std::chrono::steady_clock::now();
    try
    {
      auto grid = mapper_->project();
      const auto projection_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
      if (grid.data.empty())
      {
        return;
      }
      nav_msgs::msg::OccupancyGrid message;
      message.header.frame_id = frame_;
      message.header.stamp = now();
      message.info.resolution = resolution_;
      message.info.width = grid.width;
      message.info.height = grid.height;
      message.info.origin.position.x = grid.origin_x;
      message.info.origin.position.y = grid.origin_y;
      message.info.origin.orientation.w = 1.0;
      message.data = std::move(grid.data);
      map_pub_->publish(message);
      ++published_;
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header = message.header;
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "lidar_mapping";
      status.message = "SESSION_MAP";
      auto add = [&status](const std::string & key, const std::string & text)
      {
        diagnostic_msgs::msg::KeyValue value;
        value.key = key;
        value.value = text;
        status.values.push_back(value);
      };
      add("integrated", std::to_string(integrated_));
      add("throttled", std::to_string(throttled_));
      add("unpaired", std::to_string(unpaired_));
      for (const auto & missing : missing_)
      {
        add("missing_" + missing.first, std::to_string(missing.second));
      }
      for (const auto & received : received_)
      {
        add("received_" + received.first, std::to_string(received.second));
      }
      add("pending_frames", std::to_string(pending_.size()));
      add("tf_dropped", std::to_string(tf_dropped_));
      add("rejected", std::to_string(rejected_));
      add("published", std::to_string(published_));
      add("integration_ms", std::to_string(integration_ms_));
      add("octree_nodes", std::to_string(mapper_->nodes()));
      add("grid_width", std::to_string(grid.width));
      add("grid_height", std::to_string(grid.height));
      add("last_integrated_stamp_ns", std::to_string(last_stamp_));
      add("projection_ms", std::to_string(projection_ms));
      add("projection_publish_ms", std::to_string(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count()));
      array.status.push_back(status);
      diagnostics_pub_->publish(array);
    }
    catch (const std::exception & error)
    {
      RCLCPP_ERROR(get_logger(), "Global projection failed: %s", error.what());
    }
  }

  std::unique_ptr<OccupancyMapper> mapper_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscriptions_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::map<int64_t, Pending> pending_;
  std::map<std::string, std::size_t> missing_;
  std::map<std::string, std::size_t> received_;
  std::size_t max_pending_frames_ = 16;
  std::string frame_, sensor_, output_;
  double resolution_ = 0.1;
  int64_t integration_interval_ns_ = 500000000;
  int64_t last_stamp_ = 0;
  std::size_t integrated_ = 0, throttled_ = 0, unpaired_ = 0, tf_dropped_ = 0, rejected_ = 0, published_ = 0;
  double integration_ms_ = 0.0;
};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lidar_mapping::MappingNode>();
  rclcpp::spin(node);
  node->save_map();
  node.reset();
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
  return 0;
}
