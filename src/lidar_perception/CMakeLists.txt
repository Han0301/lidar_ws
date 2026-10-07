cmake_minimum_required(VERSION 3.16)
project(lidar_perception)
# 统一使用 C++17 编译自写节点与感知算法
set(CMAKE_CXX_STANDARD 17)
# 查找 ROS 消息、TF、Eigen 和 PCL 依赖
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search kdtree)
# 独立编译第三方地面分割核心，保持上游源码不变
add_library(patchwork_core STATIC vendor/patchworkpp/cpp/patchworkpp/src/patchworkpp.cpp vendor/patchworkpp/cpp/common/src/plane_fit.cpp)
target_include_directories(patchwork_core PUBLIC vendor/patchworkpp/cpp/patchworkpp/include vendor/patchworkpp/cpp/common/include)
target_link_libraries(patchwork_core PUBLIC Eigen3::Eigen)
# 将不依赖 ROS 接口的处理流水线编为可供节点和测试复用的库
add_library(perception_core STATIC src/cloud_pipeline.cpp src/operator_filter.cpp src/ground_segmenter.cpp src/cluster_extractor.cpp)
target_include_directories(perception_core PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(perception_core PUBLIC patchwork_core ${PCL_LIBRARIES})
# ROS 节点只负责消息收发、TF、参数和诊断，算法由核心库处理
add_executable(perception_node src/main.cpp src/perception_node.cpp)
target_link_libraries(perception_node perception_core)
ament_target_dependencies(perception_node rclcpp sensor_msgs geometry_msgs std_msgs diagnostic_msgs visualization_msgs vision_msgs tf2_ros pcl_conversions)
# 安装节点以及 YAML、launch、RViz 资源，供 ros2 launch 使用
install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
# 编译合成点云回归测试，检查地面选择、调平和射线终点保护
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pipeline test/test_pipeline.cpp)
  target_link_libraries(test_pipeline perception_core)
endif()
# 随安装资源保留自写代码和第三方组件的许可证与来源
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(FILES vendor/patchworkpp/LICENSE vendor/patchworkpp/UPSTREAM.md DESTINATION share/${PROJECT_NAME}/vendor/patchworkpp)
# 导出 ROS 包元数据并生成环境索引
ament_package()
