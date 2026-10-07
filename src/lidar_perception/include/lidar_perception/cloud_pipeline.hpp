#pragma once
#include "lidar_perception/ground_segmenter.hpp"
#include "lidar_perception/operator_filter.hpp"
#include "lidar_perception/cluster_extractor.hpp"
#include <pcl/filters/voxel_grid.h>

namespace lidar_perception
{
class CloudPipeline
{
public:
  explicit CloudPipeline(const Parameters & parameters);
  Result process(const Cloud & body_cloud, const Eigen::Affine3f & level_from_body,
    const std::vector<ExclusionBox> & exclusions);
private:
  Parameters parameters_;
  GroundSegmenter ground_;
  OperatorFilter operators_;
  ClusterExtractor clusters_;
};
}
