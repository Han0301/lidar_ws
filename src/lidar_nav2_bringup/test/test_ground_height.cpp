#include "lidar_nav2_bringup/ground_height_filter.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <vector>

TEST(GroundHeight, FirstFloorHasNoStartupAltitudeAssumption)
{
  std::vector<double> floors;
  floors.push_back(-2.1);
  floors.push_back(-1.3);
  floors.push_back(-0.7);
  floors.push_back(0.8);
  for (const double floor : floors)
  {
    lidar_nav2_bringup::GroundHeightFilter filter;
    ASSERT_TRUE(filter.update(floor, 0.5));
    EXPECT_TRUE(filter.initialized());
    EXPECT_DOUBLE_EQ(filter.height(), floor);
    EXPECT_TRUE(filter.update(floor + 0.02, 0.5));
    EXPECT_NEAR(filter.height(), floor + 0.002, 1e-9);
  }
}
TEST(GroundHeight, RejectsInvalidFloorAndReportsAbruptChange)
{
  lidar_nav2_bringup::GroundHeightFilter filter;
  EXPECT_FALSE(filter.update(std::numeric_limits<double>::quiet_NaN(), 0.5));
  EXPECT_FALSE(filter.initialized());
  ASSERT_TRUE(filter.update(-1.3, 0.5));
  EXPECT_FALSE(filter.update(-0.5, 0.5));
  EXPECT_DOUBLE_EQ(filter.height(), -1.3);
}
