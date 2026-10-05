#include <memory>

#include "mid360_monitor/mid360_monitor_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mid360_monitor::Mid360MonitorNode>());
  rclcpp::shutdown();
  return 0;
}
