#include "lidar_perception/perception_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_perception::PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
