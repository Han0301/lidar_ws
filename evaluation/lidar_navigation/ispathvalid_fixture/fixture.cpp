// Evaluation-only loader. IsPathValid is the unmodified installed Nav2 service.
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>
#include <nav2_planner/planner_server.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <std_srvs/srv/trigger.hpp>

class SnapshotPlanner : public nav2_planner::PlannerServer
{
public:
  explicit SnapshotPlanner(const std::filesystem::path & root)
  : nav2_planner::PlannerServer(rclcpp::NodeOptions()), root_(root)
  {
  }

  void load()
  {
    std::ifstream active(root_ / "active_scene.txt");
    std::string name;
    active >> name;
    if (name.empty() || name.find('/') != std::string::npos)
    {
      throw std::runtime_error("Invalid scene name");
    }
    const auto scene = root_ / name;
    unsigned int width, height;
    double resolution, origin_x, origin_y, robot_x, robot_y;
    std::ifstream meta(scene / "metadata.txt");
    if (!(meta >> width >> height >> resolution >> origin_x >> origin_y >> robot_x >> robot_y))
    {
      throw std::runtime_error("Scene metadata missing");
    }
    std::vector<unsigned char> data(static_cast<size_t>(width) * height);
    std::ifstream file(scene / "grid.bin", std::ios::binary);
    file.read(reinterpret_cast<char *>(data.data()), data.size());
    if (static_cast<size_t>(file.gcount()) != data.size())
    {
      throw std::runtime_error("Grid length mismatch");
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "nav_base";
    transform.header.stamp = now();
    transform.transform.translation.x = robot_x;
    transform.transform.translation.y = robot_y;
    transform.transform.rotation.w = 1.0;
    tf_->setTransform(transform, "snapshot_fixture", true);
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose))
    {
      throw std::runtime_error("Snapshot robot pose is unavailable");
    }
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
    costmap_->resizeMap(width, height, resolution, origin_x, origin_y);
    std::copy(data.begin(), data.end(), costmap_->getCharMap());
  }

  void enable_loader()
  {
    if (!costmap_ros_->getUseRadius())
    {
      throw std::runtime_error("This audit requires the saved circular footprint configuration");
    }
    // Freeze map updates after activation; the official checker remains unchanged.
    costmap_ros_->pause();
    loader_ = create_service<std_srvs::srv::Trigger>("load_scene",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        try
        {
          load();
          response->success = true;
          response->message = "Raw snapshot loaded";
        }
        catch (const std::exception & error)
        {
          response->success = false;
          response->message = error.what();
        }
      });
  }

private:
  std::filesystem::path root_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr loader_;
};

int main(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: ispathvalid_fixture SCENES --ros-args --params-file YAML\n";
    return 1;
  }
  const std::filesystem::path root(argv[1]);
  rclcpp::init(argc, argv);
  auto planner = std::make_shared<SnapshotPlanner>(root);
  if (planner->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    throw std::runtime_error("Official planner configure failed");
  }
  planner->load();
  if (planner->activate().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    throw std::runtime_error("Official planner activation failed");
  }
  planner->enable_loader();
  rclcpp::spin(planner->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
