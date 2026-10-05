cmake_minimum_required(VERSION 3.8)
project(mid360_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(mid360_monitor_node
  src/main.cpp
  src/mid360_monitor_node.cpp
  src/scan_timing.cpp
)
target_include_directories(mid360_monitor_node PRIVATE include)
ament_target_dependencies(mid360_monitor_node
  rclcpp
  sensor_msgs
  livox_ros_driver2
)

install(TARGETS mid360_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_timing
    test/test_scan_timing.cpp
    src/scan_timing.cpp
  )
  target_include_directories(test_scan_timing PRIVATE include)
endif()

ament_package()
