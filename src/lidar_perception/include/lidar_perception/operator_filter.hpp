#pragma once
#include "lidar_perception/types.hpp"

namespace lidar_perception
{
class OperatorFilter
{
public:
  bool excludes(const Point & point, const std::vector<ExclusionBox> & boxes) const;
};
}
