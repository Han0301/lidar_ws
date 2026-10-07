#include "lidar_nav2_bringup/lio_nav_bridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_nav2_bringup::LioNavBridge>());
  rclcpp::shutdown();
  return 0;
}
