#include "lidar_perception/cloud_pipeline.hpp"

namespace lidar_perception
{
CloudPipeline::CloudPipeline(const Parameters & parameters)
: parameters_(parameters), ground_(parameters), clusters_(parameters)
{
}

Result CloudPipeline::process(const Cloud & body_cloud, const Eigen::Affine3f & level_from_body,
  const std::vector<ExclusionBox> & exclusions)
{
  const auto began = std::chrono::steady_clock::now();
  Result result;
  result.input_points = body_cloud.size();
  auto marking = std::make_shared<Cloud>();
  auto clearing = std::make_shared<Cloud>();
  for (const auto & source : body_cloud)
  {
    if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z))
    {
      continue;
    }
    const double range = source.getVector3fMap().norm();
    if (range < parameters_.min_range || range > parameters_.max_range)
    {
      continue;
    }
    ++result.valid_points;
    Point leveled = source;
    leveled.getVector3fMap() = level_from_body * source.getVector3fMap();
    clearing->push_back(leveled);
    if (operators_.excludes(source, exclusions))
    {
      ++result.operator_points;
      result.removed->push_back(leveled);
    }
    else
    {
      marking->push_back(leveled);
    }
  }
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(parameters_.voxel_size, parameters_.voxel_size, parameters_.voxel_size);
  auto reduced = std::make_shared<Cloud>();
  voxel.setInputCloud(marking);
  voxel.filter(*reduced);
  // Clearing retains genuine operator endpoints, so rays never extend through occluders.
  *result.clearing = *clearing;
  ground_.segment(reduced, result);
  result.objects = clusters_.extract(result.obstacles);
  result.processing_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - began).count();
  return result;
}
}
