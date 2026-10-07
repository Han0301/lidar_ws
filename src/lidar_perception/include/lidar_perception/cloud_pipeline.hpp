#pragma once
#include "lidar_perception/ground_segmenter.hpp"
#include "lidar_perception/operator_filter.hpp"
#include "lidar_perception/cluster_extractor.hpp"
#include <pcl/filters/voxel_grid.h>

namespace lidar_perception
{
// 组织有效点筛选、坐标调平、区域排除、降采样、地面分割与聚类
class CloudPipeline
{
public:
  // 构造处理流水线，并将参数传给地面与聚类模块
  explicit CloudPipeline(const Parameters & parameters);
  // 处理一帧 body 点云；level_from_body 将输入点变到调平坐标，exclusions 定义排除盒
  Result process
  (
    const Cloud & body_cloud,
    const Eigen::Affine3f & level_from_body,
    const std::vector<ExclusionBox> & exclusions
  );
private:
  Parameters parameters_;      // 本流水线使用的固定参数副本
  GroundSegmenter ground_;      // 估计参考地面并分离地面、障碍点
  OperatorFilter operators_;      // 检查点是否落入已配置的空间排除区域
  ClusterExtractor clusters_;      // 为障碍点生成几何簇与包围框
};
}
