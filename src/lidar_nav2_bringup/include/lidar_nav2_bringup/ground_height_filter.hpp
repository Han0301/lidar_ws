#pragma once
#include <cmath>

namespace lidar_nav2_bringup
{
// 用实测地面建立高度基准；不把启动时的零高度当成真实地面
class GroundHeightFilter
{
public:
  // 接受地面高度观测（m）；首次直接建立基准，之后拒绝大跳变并平滑更新
  bool update(double measurement, double max_innovation)
  {
    // 非有限数值不进入高度状态
    if (!std::isfinite(measurement))
    {
      return false;
    }
    // 首次观测不受启动零高度限制，支持手持雷达从不同高度启动
    if (!initialized_)
    {
      height_ = measurement;
      initialized_ = true;
      return true;
    }
    // 后续观测与旧高度相差过大时保留旧值，由上层报告跳变
    if (std::abs(measurement - height_) > max_innovation)
    {
      return false;
    }
    // 采用旧值 90%、新观测 10% 的一阶平滑
    height_ = 0.9 * height_ + 0.1 * measurement;
    return true;
  }
  // 查询是否已建立实测地面基准
  bool initialized() const
  {
    return initialized_;
  }
  // 返回当前地面高度；未初始化时返回初始零值
  double height() const
  {
    return height_;
  }
private:
  bool initialized_ = false; // 是否已经接受第一条可信地面观测
  double height_ = 0.0; // odom 坐标系下的滤波地面 Z 高度（m）
};
}
