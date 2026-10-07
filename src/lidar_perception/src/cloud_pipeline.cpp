#include "lidar_perception/cloud_pipeline.hpp"

namespace lidar_perception
{
// 构造流水线，并以同一参数集初始化地面分割与聚类模块
CloudPipeline::CloudPipeline(const Parameters & parameters) : parameters_(parameters), ground_(parameters), clusters_(parameters)
{
}

// 将一帧 body 点云调平后分类；排除盒只作用于标记支路，清除支路保留真实回波
Result CloudPipeline::process
(
  const Cloud & body_cloud,
  const Eigen::Affine3f & level_from_body,
  const std::vector<ExclusionBox> & exclusions
)
{
  const auto began = std::chrono::steady_clock::now();      // 单帧处理起点，用单调墙钟统计耗时
  Result result;      // 本帧点云、几何框、地面状态和统计的输出集合
  result.input_points = body_cloud.size();
  auto marking = std::make_shared<Cloud>();      // 经过区域排除后参与地面/障碍分类的调平点云
  auto clearing = std::make_shared<Cloud>();      // 区域排除前的真实观测终点，避免向遮挡物后方补射线
  // 逐点检查有限值和距离，再分别构造标记与清除输入
  for (const auto & source : body_cloud)
  {
    if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z))
    {
      continue;
    }
    const double range = source.getVector3fMap().norm();      // 点到 body 原点的三维距离（m）
    if (range < parameters_.min_range || range > parameters_.max_range)
    {
      continue;
    }
    ++result.valid_points;
    Point leveled = source;      // 保留原始强度，仅将坐标变到调平参考系
    // 应用同一扫描时刻 TF，让地面高度判断不随手持雷达倾斜改变
    leveled.getVector3fMap() = level_from_body * source.getVector3fMap();
    clearing->push_back(leveled);
    // 排除盒定义在 body 中，包含关系使用原始点；删除结果仍以调平坐标输出
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
  // 仅对分类支路降采样，降低平面拟合与聚类开销
  pcl::VoxelGrid<Point> voxel;      // 以小体素内的代表点减少后续计算量
  voxel.setLeafSize(parameters_.voxel_size, parameters_.voxel_size, parameters_.voxel_size);
  auto reduced = std::make_shared<Cloud>();      // 降采样后的分类输入，不用于替换清除终点
  voxel.setInputCloud(marking);
  voxel.filter(*reduced);
  // 清除支路保留被区域过滤的真实终点，不把“删除障碍点”误当成“看到了更远处”
  *result.clearing = *clearing;
  // 先建立可信地面参考，再分离地面和高度范围内的非地面点
  ground_.segment(reduced, result);
  // 对障碍点生成可视化包围框；地图标记仍直接使用障碍点云
  result.objects = clusters_.extract(result.obstacles);
  // 记录流水线实际耗时，用于诊断，不使用可能暂停的回放时钟
  result.processing_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - began).count();
  return result;
}
}
