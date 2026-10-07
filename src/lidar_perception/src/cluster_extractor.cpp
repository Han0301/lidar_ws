#include "lidar_perception/cluster_extractor.hpp"

namespace lidar_perception
{
ClusterExtractor::ClusterExtractor(const Parameters & parameters)
: parameters_(parameters)
{
}

std::vector<Object> ClusterExtractor::extract(const Cloud::ConstPtr & cloud) const
{
  std::vector<Object> objects;
  if (cloud->empty())
  {
    return objects;
  }
  auto tree = std::make_shared<pcl::search::KdTree<Point>>();
  tree->setInputCloud(cloud);
  pcl::EuclideanClusterExtraction<Point> extraction;
  extraction.setClusterTolerance(parameters_.cluster_tolerance);
  extraction.setMinClusterSize(parameters_.cluster_min_points);
  extraction.setMaxClusterSize(parameters_.cluster_max_points);
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(cloud);
  std::vector<pcl::PointIndices> clusters;
  extraction.extract(clusters);
  for (const auto & cluster : clusters)
  {
    Eigen::Vector3f minimum = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector3f maximum = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
    for (const int index : cluster.indices)
    {
      minimum = minimum.cwiseMin(cloud->at(index).getVector3fMap());
      maximum = maximum.cwiseMax(cloud->at(index).getVector3fMap());
    }
    Object object;
    object.center = 0.5F * (minimum + maximum);
    object.size = maximum - minimum;
    object.points = cluster.indices.size();
    objects.push_back(object);
  }
  return objects;
}
}
