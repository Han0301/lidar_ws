#pragma once
#include "lidar_perception/types.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <patchwork/patchworkpp.h>

namespace lidar_perception
{
// 先建立低处参考平面，再约束局部分割结果，避免把桌面当成地面
class GroundSegmenter
{
public:
  // 初始化局部地面分割器及其归一化参考参数
  explicit GroundSegmenter(const Parameters & parameters);
  // 将调平、降采样后的点写入 result 的地面/障碍集合，同时更新可信标志
  void segment(const Cloud::ConstPtr & cloud, Result & result);
private:
  Parameters parameters_;      // 平面距离、地面方法和障碍高度等配置
  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;      // 复用的 Patchwork++ 分割器，避免逐帧创建
  double last_height_ = 0.5;      // 最近一次可信的垂直地面高度（m）；初值不能替代本帧可信判断
};
}
