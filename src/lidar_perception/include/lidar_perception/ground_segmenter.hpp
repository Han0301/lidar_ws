#pragma once
#include "lidar_perception/types.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <patchwork/patchworkpp.h>

namespace lidar_perception
{
class GroundSegmenter
{
public:
  explicit GroundSegmenter(const Parameters & parameters);
  void segment(const Cloud::ConstPtr & cloud, Result & result);
private:
  Parameters parameters_;
  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
  double last_height_ = 0.5;
};
}
