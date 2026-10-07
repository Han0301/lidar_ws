#include "lidar_nav2_bringup/ground_height_filter.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <vector>

// 验证首个有效高度直接建基准，后续小变化按 0.1 权重滤波
TEST(GroundHeight, FirstFloorHasNoStartupAltitudeAssumption)
{
  std::vector<double> floors; // 不同初始绝对地面高度（m），覆盖零点上下
  floors.push_back(-2.1);
  floors.push_back(-1.3);
  floors.push_back(-0.7);
  floors.push_back(0.8);
  // 对多个绝对高度分别验证初始化，避免默认从零高度做跳变判断
  for (const double floor : floors)
  {
    lidar_nav2_bringup::GroundHeightFilter filter; // 每组独立的地面高度状态
    ASSERT_TRUE(filter.update(floor, 0.5));
    EXPECT_TRUE(filter.initialized());
    EXPECT_DOUBLE_EQ(filter.height(), floor);
    EXPECT_TRUE(filter.update(floor + 0.02, 0.5));
    EXPECT_NEAR(filter.height(), floor + 0.002, 1e-9);
  }
}
// 验证无效数值不初始化，超过门限的地面跳变被拒绝并保留旧高度
TEST(GroundHeight, RejectsInvalidFloorAndReportsAbruptChange)
{
  lidar_nav2_bringup::GroundHeightFilter filter; // 每组独立的地面高度状态
  EXPECT_FALSE(filter.update(std::numeric_limits<double>::quiet_NaN(), 0.5));
  EXPECT_FALSE(filter.initialized());
  ASSERT_TRUE(filter.update(-1.3, 0.5));
  EXPECT_FALSE(filter.update(-0.5, 0.5));
  EXPECT_DOUBLE_EQ(filter.height(), -1.3);
}
