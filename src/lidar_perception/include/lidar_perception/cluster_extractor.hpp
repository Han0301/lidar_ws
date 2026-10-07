#pragma once
#include "lidar_perception/types.hpp"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

namespace lidar_perception
{
// 对非地面点做欧氏聚类，并输出轴对齐几何包围框
class ClusterExtractor
{
public:
  // 保存聚类配置
  explicit ClusterExtractor(const Parameters & parameters);
  // 从当前障碍点云提取几何簇；不修改输入，也不跟踪历史身份
  std::vector<Object> extract(const Cloud::ConstPtr & cloud) const;
private:
  Parameters parameters_;      // 聚类连接距离以及每个簇的点数上下限
};
}
