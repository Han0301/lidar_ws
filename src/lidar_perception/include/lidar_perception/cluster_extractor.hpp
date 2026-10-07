#pragma once
#include "lidar_perception/types.hpp"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

namespace lidar_perception
{
class ClusterExtractor
{
public:
  explicit ClusterExtractor(const Parameters & parameters);
  std::vector<Object> extract(const Cloud::ConstPtr & cloud) const;
private:
  Parameters parameters_;
};
}
