#include "lidar_perception/cluster_extractor.hpp"

namespace lidar_perception
{
// 保存欧氏连接距离和簇点数限制
ClusterExtractor::ClusterExtractor(const Parameters & parameters) : parameters_(parameters)
{
}

// 从当前障碍点云提取几何簇；小簇不出框，但其点仍留在障碍地图输入中
std::vector<Object> ClusterExtractor::extract(const Cloud::ConstPtr & cloud) const
{
  std::vector<Object> objects;      // 本帧满足点数限制的几何簇输出
  if (cloud->empty())
  {
    return objects;
  }
  // 建立搜索树并执行一次欧氏聚类
  auto tree = std::make_shared<pcl::search::KdTree<Point>>();      // 加速障碍点的距离邻域搜索
  // 欧氏聚类器会建立搜索树，避免重复构建同一帧
  pcl::EuclideanClusterExtraction<Point> extraction;      // 把距离可连接的点归为同一几何簇
  extraction.setClusterTolerance(parameters_.cluster_tolerance);
  extraction.setMinClusterSize(parameters_.cluster_min_points);
  extraction.setMaxClusterSize(parameters_.cluster_max_points);
  extraction.setSearchMethod(tree);
  extraction.setInputCloud(cloud);
  std::vector<pcl::PointIndices> clusters;      // 各个簇在原障碍点云中的索引集合
  extraction.extract(clusters);
  // 逐簇计算坐标范围，得到中心、全边长和点数
  for (const auto & cluster : clusters)
  {
    Eigen::Vector3f minimum = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());      // 当前簇沿 X/Y/Z 的最小坐标（m）
    Eigen::Vector3f maximum = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());      // 当前簇沿 X/Y/Z 的最大坐标（m）
    for (const int index : cluster.indices)
    {
      minimum = minimum.cwiseMin(cloud->at(index).getVector3fMap());
      maximum = maximum.cwiseMax(cloud->at(index).getVector3fMap());
    }
    Object object;      // 由坐标范围构造的轴对齐包围框，无语义类别
    object.center = 0.5F * (minimum + maximum);
    object.size = maximum - minimum;
    object.points = cluster.indices.size();
    objects.push_back(object);
  }
  return objects;
}
}
