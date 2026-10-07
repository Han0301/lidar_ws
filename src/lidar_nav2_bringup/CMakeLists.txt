cmake_minimum_required(VERSION 3.16)
project(lidar_nav2_bringup)
# 统一使用 C++17 编译坐标桥接节点
set(CMAKE_CXX_STANDARD 17)
# 查找 ROS 消息、TF 和 Eigen 依赖
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# 构建 LIO→导航坐标桥接节点，不改动驱动或 FAST-LIO2
add_executable(lio_nav_bridge src/main.cpp src/lio_nav_bridge.cpp)
target_include_directories(lio_nav_bridge PRIVATE include)
target_link_libraries(lio_nav_bridge Eigen3::Eigen)
ament_target_dependencies(lio_nav_bridge rclcpp sensor_msgs geometry_msgs nav_msgs diagnostic_msgs tf2_ros)
# 安装节点和启动所需的参数、地图、RViz 资源
install(TARGETS lio_nav_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch maps rviz DESTINATION share/${PROJECT_NAME})
# 把 Python 启动门控安装为 ros2 可启动的可执行程序
install(PROGRAMS scripts/nav2_startup_gate.py DESTINATION lib/${PROJECT_NAME} RENAME nav2_startup_gate)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
# 编译地面高度状态测试，检查初始基准与异常跳变拒绝
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ground_height test/test_ground_height.cpp)
  target_include_directories(test_ground_height PRIVATE include)
endif()
# 导出 ROS 包元数据并生成环境索引
ament_package()
