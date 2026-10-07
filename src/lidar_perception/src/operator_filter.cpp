#include "lidar_perception/operator_filter.hpp"

namespace lidar_perception
{
// 判断点是否属于任一指定空间盒；只做区域包含检查，不识别人
bool OperatorFilter::excludes(const Point & point, const std::vector<ExclusionBox> & boxes) const
{
  // 变换到每个盒的局部坐标，三个轴同时在半边长范围内即命中
  for (const auto & box : boxes)
  {
    const Eigen::Vector3f local = box.box_from_body * point.getVector3fMap();      // 原始 body 点在当前排除盒局部坐标中的位置（m）
    if ((local.array().abs() <= box.half_size.array()).all())
    {
      return true;
    }
  }
  return false;
}
}
