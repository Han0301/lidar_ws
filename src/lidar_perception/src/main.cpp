#include "lidar_perception/perception_node.hpp"

// 初始化 ROS 2，运行感知节点，退出时释放 ROS 资源
int main(int argc, char ** argv)
{
  // 解析 ROS 参数并建立进程上下文
  rclcpp::init(argc, argv);
  // 持续处理订阅和定时器回调，直到收到退出信号
  rclcpp::spin(std::make_shared<lidar_perception::PerceptionNode>());
  // 关闭上下文并释放通信资源
  rclcpp::shutdown();
  return 0;
}
