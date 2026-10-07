#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_perception
{
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;

struct Parameters
{
  double min_range = 0.45;
  double max_range = 15.0;
  double voxel_size = 0.10;
  double ground_distance = 0.08;
  double ground_guard_distance = 0.16;
  double obstacle_min_height = 0.12;
  double obstacle_max_height = 2.50;
  double cluster_tolerance = 0.30;
  int cluster_min_points = 6;
  int cluster_max_points = 30000;
  std::string ground_method = "patchwork";
};

struct ExclusionBox
{
  Eigen::Affine3f box_from_body = Eigen::Affine3f::Identity();
  Eigen::Vector3f half_size = Eigen::Vector3f::Zero();
  std::string id;
};

struct Object
{
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f size = Eigen::Vector3f::Zero();
  std::size_t points = 0;
};

struct Result
{
  Cloud::Ptr ground = std::make_shared<Cloud>();
  Cloud::Ptr obstacles = std::make_shared<Cloud>();
  Cloud::Ptr clearing = std::make_shared<Cloud>();
  Cloud::Ptr removed = std::make_shared<Cloud>();
  std::vector<Object> objects;
  std::size_t input_points = 0;
  std::size_t valid_points = 0;
  std::size_t operator_points = 0;
  double sensor_height = 0.0;
  bool ground_reference_valid = false;
  double processing_ms = 0.0;
};
}
