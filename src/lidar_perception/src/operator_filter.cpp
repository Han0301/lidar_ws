#include "lidar_perception/operator_filter.hpp"

namespace lidar_perception
{
bool OperatorFilter::excludes(const Point & point, const std::vector<ExclusionBox> & boxes) const
{
  for (const auto & box : boxes)
  {
    const Eigen::Vector3f local = box.box_from_body * point.getVector3fMap();
    if ((local.array().abs() <= box.half_size.array()).all())
    {
      return true;
    }
  }
  return false;
}
}
