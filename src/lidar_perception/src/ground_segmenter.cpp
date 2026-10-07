#include "lidar_perception/ground_segmenter.hpp"

namespace lidar_perception
{
GroundSegmenter::GroundSegmenter(const Parameters & parameters)
: parameters_(parameters)
{
  patchwork::Params settings;
  settings.min_range = parameters.min_range;
  settings.max_range = parameters.max_range;
  settings.sensor_height = 1.3;
  settings.num_min_pts = 5;
  settings.th_dist = parameters.ground_distance;
  settings.th_seeds = 0.15;
  settings.enable_RNR = false;
  settings.verbose = false;
  patchwork_ = std::make_unique<patchwork::PatchWorkpp>(settings);
}

void GroundSegmenter::segment(const Cloud::ConstPtr & cloud, Result & result)
{
  if (cloud->empty())
  {
    return;
  }
  auto candidates = std::make_shared<Cloud>();
  for (const auto & point : *cloud)
  {
    const double range = std::hypot(point.x, point.y);
    if (range > 1.0 && range < 7.0 && point.z < 0.25 && point.z > -3.5)
    {
      candidates->push_back(point);
    }
  }
  Eigen::Vector3f normal(0.0F, 0.0F, 1.0F);
  double plane_d = last_height_;
  if (candidates->size() >= 80)
  {
    // Fit the low surface first: a larger tabletop must not become the floor.
    std::vector<float> heights;
    for (const auto & point : *candidates)
    {
      heights.push_back(point.z);
    }
    const auto quantile = heights.begin() + heights.size() / 10;
    std::nth_element(heights.begin(), quantile, heights.end());
    const float ceiling = *quantile + 0.30F;
    auto low_candidates = std::make_shared<Cloud>();
    for (const auto & point : *candidates)
    {
      if (point.z <= ceiling)
      {
        low_candidates->push_back(point);
      }
    }
    pcl::SACSegmentation<Point> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(0.30);
    segmentation.setDistanceThreshold(parameters_.ground_distance);
    segmentation.setMaxIterations(100);
    segmentation.setInputCloud(low_candidates);
    pcl::PointIndices indices;
    pcl::ModelCoefficients coefficients;
    segmentation.segment(indices, coefficients);
    if (indices.indices.size() >= 60 && coefficients.values.size() == 4)
    {
      normal = Eigen::Vector3f(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
      const float norm = normal.norm();
      const float sign = normal.z() < 0.0F ? -1.0F : 1.0F;
      normal *= sign / norm;
      plane_d = coefficients.values[3] * sign / norm;
      const double height = plane_d / normal.z();
      result.ground_reference_valid = normal.z() > 0.95 && height > -0.20 && height < 3.0;
      if (result.ground_reference_valid)
      {
        last_height_ = height;
      }
    }
  }
  result.sensor_height = last_height_;
  if (!result.ground_reference_valid)
  {
    // Unknown ground: keep observations as obstacles rather than invent free space.
    *result.obstacles = *cloud;
    return;
  }
  std::vector<bool> is_ground(cloud->size(), false);
  if (parameters_.ground_method == "patchwork")
  {
    Eigen::MatrixXf points(cloud->size(), 4);
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
      const auto & point = cloud->at(i);
      points(i, 0) = point.x;
      points(i, 1) = point.y;
      points(i, 2) = point.z + last_height_ - 1.3;
      points(i, 3) = point.intensity;
    }
    patchwork_->estimateGround(points);
    const auto ground_indices = patchwork_->getGroundIndices();
    for (Eigen::Index i = 0; i < ground_indices.size(); ++i)
    {
      const int index = ground_indices[i];
      if (index >= 0 && static_cast<std::size_t>(index) < is_ground.size())
      {
        is_ground[index] = true;
      }
    }
  }
  for (std::size_t i = 0; i < cloud->size(); ++i)
  {
    const auto & point = cloud->at(i);
    const double signed_distance = normal.dot(point.getVector3fMap()) + plane_d;
    // Patchwise horizontal roofs or tabletops are not traversable ground.
    if (is_ground[i] && std::abs(signed_distance) > parameters_.ground_guard_distance)
    {
      is_ground[i] = false;
    }
    if (result.ground_reference_valid &&
      (parameters_.ground_method == "plane" || std::hypot(point.x, point.y) < 1.2))
    {
      is_ground[i] = std::abs(signed_distance) < parameters_.ground_distance;
    }
    if (is_ground[i])
    {
      result.ground->push_back(point);
    }
    else if (point.z + last_height_ >= parameters_.obstacle_min_height &&
      point.z + last_height_ <= parameters_.obstacle_max_height)
    {
      result.obstacles->push_back(point);
    }
  }
}
}
