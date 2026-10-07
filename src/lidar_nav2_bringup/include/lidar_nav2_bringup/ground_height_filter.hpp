#pragma once
#include <cmath>

namespace lidar_nav2_bringup
{
// The first measured floor defines the datum; startup altitude is not a constraint.
class GroundHeightFilter
{
public:
  bool update(double measurement, double max_innovation)
  {
    if (!std::isfinite(measurement))
    {
      return false;
    }
    if (!initialized_)
    {
      height_ = measurement;
      initialized_ = true;
      return true;
    }
    if (std::abs(measurement - height_) > max_innovation)
    {
      return false;
    }
    height_ = 0.9 * height_ + 0.1 * measurement;
    return true;
  }
  bool initialized() const
  {
    return initialized_;
  }
  double height() const
  {
    return height_;
  }
private:
  bool initialized_ = false;
  double height_ = 0.0;
};
}
