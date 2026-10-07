#include "lidar_perception/ground_segmenter.hpp"

namespace lidar_perception
{
// 初始化 Patchwork++；固定参考高度仅用于输入归一化，不替代实测高度
GroundSegmenter::GroundSegmenter(const Parameters & parameters) : parameters_(parameters)
{
  patchwork::Params settings;      // 第三方局部分割器的配置；实际地面高度由本帧点云估计
  settings.min_range = parameters.min_range;
  settings.max_range = parameters.max_range;
  settings.sensor_height = 1.3;      // Patchwork++ 输入归一化的参考高度（m），不是固定安装高度
  settings.num_min_pts = 5;      // 局部区域进行分割所需的最少点数
  settings.th_dist = parameters.ground_distance;      // 局部点到地面模型的距离阈值（m）
  settings.th_seeds = 0.15;      // 选取局部低处种子点的高度容差（m）
  settings.enable_RNR = false;      // 关闭额外反射噪声处理，保持本配置的分割行为
  settings.verbose = false;
  patchwork_ = std::make_unique<patchwork::PatchWorkpp>(settings);
}

// 从调平点云估计可信参考地面，再分割地面与障碍并写入 result
void GroundSegmenter::segment(const Cloud::ConstPtr & cloud, Result & result)
{
  if (cloud->empty())
  {
    return;
  }
  // 只从近场低处区域拟合参考，减少远处稀疏点和高处物体干扰
  auto candidates = std::make_shared<Cloud>();      // 用于估计整体参考地面的近场、低处点
  for (const auto & point : *cloud)
  {
    const double range = std::hypot(point.x, point.y);      // 点在水平面的距离（m），不同于流水线的三维距离
    if (range > 1.0 && range < 7.0 && point.z < 0.25 && point.z > -3.5)
    {
      candidates->push_back(point);
    }
  }
  Eigen::Vector3f normal(0.0F, 0.0F, 1.0F);      // 归一化且朝上的参考平面法向量
  double plane_d = last_height_;      // 平面 normal·point + plane_d = 0 的常数项（m）
  // 候选足够时才拟合本帧参考；历史高度本身不能使本帧变可信
  if (candidates->size() >= 80)
  {
    // 优先拟合低处点群，避免点数更多的桌面抢占地面参考
    std::vector<float> heights;      // 候选点的 Z 值，用低分位筛选地面种子
    for (const auto & point : *candidates)
    {
      heights.push_back(point.z);
    }
    const auto quantile = heights.begin() + heights.size() / 10;      // 约第 10 百分位高度的迭代器
    std::nth_element(heights.begin(), quantile, heights.end());
    const float ceiling = *quantile + 0.30F;      // 低处候选的高度上限（m），抑制更高且更大的桌面
    auto low_candidates = std::make_shared<Cloud>();      // 处于低分位高度附近的平面拟合输入
    for (const auto & point : *candidates)
    {
      if (point.z <= ceiling)
      {
        low_candidates->push_back(point);
      }
    }
    // 寻找法向接近重力 Z 轴的平面，排除垂直墙面
    pcl::SACSegmentation<Point> segmentation;      // 带重力方向约束的 RANSAC 平面拟合器
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(0.30);      // 平面法向与 Z 轴允许的最大夹角（rad）
    segmentation.setDistanceThreshold(parameters_.ground_distance);
    segmentation.setMaxIterations(100);      // 限制单帧 RANSAC 的最大迭代次数
    segmentation.setInputCloud(low_candidates);
    pcl::PointIndices indices;      // 落在拟合平面上的内点索引
    pcl::ModelCoefficients coefficients;      // 平面原始系数 a、b、c、d，后面统一归一化
    segmentation.segment(indices, coefficients);
    // 内点足够且系数完整时，检查方向和高度是否在配置支持范围内
    if (indices.indices.size() >= 60 && coefficients.values.size() == 4)
    {
      normal = Eigen::Vector3f(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
      const float norm = normal.norm();      // 原始法向量长度，用于消除平面系数的比例自由度
      const float sign = normal.z() < 0.0F ? -1.0F : 1.0F;      // 把法向量统一翻到朝上的方向
      normal *= sign / norm;
      plane_d = coefficients.values[3] * sign / norm;
      const double height = plane_d / normal.z();      // 坐标原点到参考平面的垂直高度（m），不是法向距离
      // 这是参考平面的可信门控，不是机器人的坡度通行能力判断
      result.ground_reference_valid = normal.z() > 0.95 && height > -0.20 && height < 3.0;
      if (result.ground_reference_valid)
      {
        last_height_ = height;
      }
    }
  }
  result.sensor_height = last_height_;
  // 地面不确定时不虚构自由空间；节点层会暂停发布地图标记和清除输入
  if (!result.ground_reference_valid)
  {
    // 保留点云作为诊断用非地面候选；上层必须检查 ground_reference_valid
    *result.obstacles = *cloud;
    return;
  }
  std::vector<bool> is_ground(cloud->size(), false);      // 逐点地面标记，后面同时受到局部分割和参考平面约束
  // 使用实测参考高度归一化后执行局部地面分割
  if (parameters_.ground_method == "patchwork")
  {
    Eigen::MatrixXf points(cloud->size(), 4);      // Patchwork++ 所需的 N×4 输入：X、Y、归一化 Z、强度
    // 按输入索引逐点处理，保证地面标记与点云顺序一致
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
      const auto & point = cloud->at(i);      // 当前降采样点，保持与局部分割输入相同的索引
      points(i, 0) = point.x;
      points(i, 1) = point.y;
      points(i, 2) = point.z + last_height_ - 1.3;      // 先用实测高度恢复地面，再移到算法的 1.3 m 参考高度
      points(i, 3) = point.intensity;
    }
    patchwork_->estimateGround(points);
    const auto ground_indices = patchwork_->getGroundIndices();      // 局部分割认为是地面的原始输入索引
    for (Eigen::Index i = 0; i < ground_indices.size(); ++i)
    {
      const int index = ground_indices[i];      // 一个局部地面点在 cloud 中的位置
      if (index >= 0 && static_cast<std::size_t>(index) < is_ground.size())
      {
        is_ground[index] = true;
      }
    }
  }
  // 按输入索引逐点处理，保证地面标记与点云顺序一致
  for (std::size_t i = 0; i < cloud->size(); ++i)
  {
    const auto & point = cloud->at(i);      // 当前降采样点，保持与局部分割输入相同的索引
    const double signed_distance = normal.dot(point.getVector3fMap()) + plane_d;      // 当前点到参考平面的有符号法向距离（m）
    // 局部水平面还要贴近参考地面，远离地面的桌面和屋顶不能被当作地面
    if (is_ground[i] && std::abs(signed_distance) > parameters_.ground_guard_distance)
    {
      is_ground[i] = false;
    }
    // 纯平面模式和近场区域直接使用参考平面距离，弥补近处局部分割空缺
    if (result.ground_reference_valid &&
      (parameters_.ground_method == "plane" || std::hypot(point.x, point.y) < 1.2))
    {
      is_ground[i] = std::abs(signed_distance) < parameters_.ground_distance;
    }
    // 地面点单独输出；其他点仅在离地高度范围内进入障碍集合
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
