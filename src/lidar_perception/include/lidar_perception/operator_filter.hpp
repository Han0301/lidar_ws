#pragma once
#include "lidar_perception/types.hpp"

namespace lidar_perception
{
// 仅按空间盒筛选点，不执行人体识别
class OperatorFilter
{
public:
  // 检查一个 body 点是否落入任意排除盒，命中则返回 true
  bool excludes(const Point & point, const std::vector<ExclusionBox> & boxes) const;
};
}
