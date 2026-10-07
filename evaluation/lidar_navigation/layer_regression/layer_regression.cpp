#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/voxel_layer.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_ground_consistency_costmap_plugin/ground_consistency_layer.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/create_timer_ros.h>

using XYZ = std::array<float, 3>;
using Points = std::vector<XYZ>;
void point(Points & cloud, float x, float y, float z)
{
  XYZ p;
  p[0] = x;
  p[1] = y;
  p[2] = z;
  cloud.push_back(p);
}
Points floor_points(bool expose_target)
{
  Points out;
  for (float x = -3.95F; x < 4; x += .1F)
  {
    for (float y = -3.95F; y < 4; y += .1F)
    {
      if (expose_target || x < 1.9F || x > 2.4F || y < -.2F || y > .4F)
      {
        point(out, x, y, 0);
      }
    }
  }
  return out;
}
Points obstacle_points(float height)
{
  Points out;
  for (float x = 2.025F; x < 2.3F; x += .05F)
  {
    for (float y = .025F; y < .3F; y += .05F)
    {
      for (float z = .16F; z <= height + .001F; z += .2F)
      {
        point(out, x, y, z);
      }
    }
  }
  return out;
}
sensor_msgs::msg::PointCloud2 message(const Points & points, const rclcpp::Time & now)
{
  sensor_msgs::msg::PointCloud2 out;
  out.header.frame_id = "world";
  out.header.stamp = now;
  sensor_msgs::PointCloud2Modifier modifier(out);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());
  sensor_msgs::PointCloud2Iterator<float> x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> z(out, "z");
  for (const auto & p : points)
  {
    *x = p[0]; *y = p[1]; *z = p[2];
    ++x; ++y; ++z;
  }
  return out;
}
class Fixture
{
public:
  explicit Fixture(bool ground, const std::string & suffix)
  : map_("world", false, true), ground_(ground)
  {
    std::vector<rclcpp::Parameter> params;
    auto add = [&params](const auto & name, const auto & value)
    {
      params.emplace_back(name, value);
    };
    const std::string prefix = "layer.";
    const std::string topic = "/layer_regression/" + suffix;
    add(prefix + "ground_points_topic", topic + "/ground");
    add(prefix + "nonground_points_topic", topic + "/mark");
    add(prefix + "robot_height", 2.5);
    add(prefix + "min_clearance", 0.12);
    add(prefix + "ground_neighbor_search_cells", 1);
    add(prefix + "ground_decay", 0.8);
    add(prefix + "nonground_decay", 0.85);
    add(prefix + "max_score", 20.0);
    add(prefix + "ground_free_thresh", 2.0);
    add(prefix + "nonground_occ_thresh", 6.0);
    add(prefix + "observation_sources", std::string("mark clear"));
    add(prefix + "origin_z", -0.2);
    add(prefix + "z_resolution", 0.25);
    add(prefix + "z_voxels", 16);
    add(prefix + "unknown_threshold", 15);
    add(prefix + "mark_threshold", 0);
    add(prefix + "max_obstacle_height", 3.5);
    std::vector<std::string> sources;
    sources.push_back("mark"); sources.push_back("clear");
    for (const auto & source : sources)
    {
      const std::string key = prefix + source + ".";
      add(key + "topic", topic + "/" + source);
      add(key + "data_type", std::string("PointCloud2"));
      add(key + "sensor_frame", std::string("sensor"));
      add(key + "marking", source == "mark");
      add(key + "clearing", source == "clear");
      add(key + "obstacle_min_range", 0.0);
      add(key + "obstacle_max_range", 12.0);
      add(key + "raytrace_min_range", 0.0);
      add(key + "raytrace_max_range", 15.0);
      add(key + "min_obstacle_height", -0.2);
      add(key + "max_obstacle_height", 3.5);
    }
    node_ = std::make_shared<nav2_util::LifecycleNode>("layer_test_" + suffix, "",
      rclcpp::NodeOptions().parameter_overrides(params));
    publisher_ = std::make_shared<rclcpp::Node>("layer_input_" + suffix);
    ground_pub_ = publisher_->create_publisher<sensor_msgs::msg::PointCloud2>(topic + "/ground", rclcpp::SensorDataQoS());
    obstacle_pub_ = publisher_->create_publisher<sensor_msgs::msg::PointCloud2>(topic + "/mark", rclcpp::SensorDataQoS());
    clear_pub_ = publisher_->create_publisher<sensor_msgs::msg::PointCloud2>(topic + "/clear", rclcpp::SensorDataQoS());
    map_.resizeMap(100, 100, .1, -5, -5);
    map_.setFootprint(nav2_costmap_2d::makeFootprintFromRadius(.25));
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_->setCreateTimerInterface(std::make_shared<tf2_ros::CreateTimerROS>(
      node_->get_node_base_interface(), node_->get_node_timers_interface()));
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "world";
    transform.child_frame_id = "sensor";
    transform.transform.translation.z = 1.3;
    transform.transform.rotation.w = 1.0;
    tf_->setTransform(transform, "fixture", true);
    group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    if (ground_)
    {
      layer_ = std::make_shared<nav2_ground_consistency_costmap_plugin::GroundConsistencyLayer>();
    }
    else
    {
      layer_ = std::make_shared<nav2_costmap_2d::VoxelLayer>();
    }
    layer_->initialize(&map_, "layer", tf_.get(), node_, group_);
    map_.addPlugin(layer_);
    layer_->activate();
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(publisher_);
    const auto began = std::chrono::steady_clock::now();
    while ((ground_ ? ground_pub_->get_subscription_count() : clear_pub_->get_subscription_count()) < 1)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      if (std::chrono::steady_clock::now() - began > std::chrono::seconds(3))
      {
        throw std::runtime_error("Missing plugin subscriptions");
      }
    }
  }
  ~Fixture()
  {
    layer_->deactivate();
    executor_.remove_node(node_->get_node_base_interface());
    executor_.remove_node(publisher_);
  }
  double step(const Points & floor, const Points & obstacles, bool publish = true)
  {
    if (publish)
    {
      const auto stamp = publisher_->now();
      ground_pub_->publish(message(floor, stamp));
      obstacle_pub_->publish(message(obstacles, stamp));
      Points clearing = floor;
      clearing.insert(clearing.end(), obstacles.begin(), obstacles.end());
      clear_pub_->publish(message(clearing, stamp));
    }
    for (int i = 0; i < 8; ++i)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto before = std::chrono::steady_clock::now();
    map_.updateMap(0, 0, 0);
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count();
  }
  unsigned int cost(float x, float y)
  {
    unsigned int mx, my;
    if (!map_.getCostmap()->worldToMap(x, y, mx, my))
    {
      throw std::runtime_error("Cell outside fixture map");
    }
    return map_.getCostmap()->getCost(mx, my);
  }
private:
  nav2_costmap_2d::LayeredCostmap map_;
  bool ground_;
  std::shared_ptr<nav2_util::LifecycleNode> node_;
  std::shared_ptr<rclcpp::Node> publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Layer> layer_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_, obstacle_pub_, clear_pub_;
};
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::ofstream out(argv[1]);
  out << "layer,case,cycle,target_cost,update_ms\n";
  for (int type = 0; type < 2; ++type)
  {
    for (int test = 0; test < 6; ++test)
    {
      const std::string name = "case_" + std::to_string(type) + "_" + std::to_string(test);
      Fixture fixture(type == 1, name);
      for (int cycle = 0; cycle < 100; ++cycle)
      {
        Points ground;
        Points obstacles;
        if (test == 4)
        {
          // A single weak floor return does not establish a free cell.
          point(ground, 2.05F, .05F, 0);
        }
        else if (test == 5)
        {
          // Fresh short obstacles must override accumulated floor evidence.
          ground = floor_points(true);
          if (cycle >= 10)
          {
            obstacles = obstacle_points(.16F);
          }
        }
        else if (test == 0 || cycle < 10)
        {
          ground = floor_points(false);
          obstacles = obstacle_points(test == 0 ? .16F : 1.36F);
        }
        else if (test == 1)
        {
          ground = floor_points(true);
        }
        else
        {
          // New data elsewhere does not establish free space at the former obstacle.
          point(ground, 4.05F, 4.05F, 0);
        }
        const double ms = fixture.step(ground, obstacles, !(test == 3 && cycle >= 10));
        out << (type == 0 ? "voxel" : "ground") << ',' << test << ',' << cycle << ','
          << fixture.cost(2.05F, .05F) << ',' << ms << '\n';
        if (type == 1)
        {
          const auto cost = fixture.cost(2.05F, .05F);
          const bool invalid = (test == 0 && cost != 254) ||
            (test == 1 && cycle == 99 && cost != 0) ||
            (test == 2 && cycle >= 10 && cost == 0) ||
            (test == 3 && cost != 254) ||
            (test == 4 && cycle < 3 && cost != 255) ||
            (test == 5 && cycle >= 10 && cost != 254);
          if (invalid)
          {
            throw std::runtime_error("Ground layer regression failed: " + name);
          }
        }
      }
    }
  }
  rclcpp::shutdown();
}
