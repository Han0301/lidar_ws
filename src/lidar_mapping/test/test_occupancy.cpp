#include "lidar_mapping/occupancy_mapper.hpp"
#include <filesystem>
#include <gtest/gtest.h>

namespace
{
using lidar_mapping::OccupancyMapper;
using lidar_mapping::Settings;
using Point = octomap::point3d;

// 按世界坐标读取投影格，超出图幅也保持未知
int cost(const lidar_mapping::Grid & grid, const Point & point)
{
  const int x = std::floor((point.x() - grid.origin_x) / .1);
  const int y = std::floor((point.y() - grid.origin_y) / .1);
  if (x < 0 || y < 0 || x >= grid.width || y >= grid.height)
  {
    return -1;
  }
  return grid.data[y * grid.width + x];
}

TEST(Occupancy, RayWithoutGroundDoesNotInventFreeColumn)
{
  Settings settings;
  OccupancyMapper mapper(settings);
  std::vector<Point> returns;
  returns.emplace_back(3.05, .05, 1.0);
  std::vector<Point> empty;
  mapper.integrate(Point(.05, .05, 1.0), returns, empty, empty);
  EXPECT_TRUE(mapper.project().data.empty());
}

TEST(Occupancy, RepeatedGroundSupportsFreeAndMissingEvidenceRemainsUnknown)
{
  Settings settings;
  OccupancyMapper mapper(settings);
  std::vector<Point> ground;
  ground.emplace_back(2.05, .05, 0.0);
  std::vector<Point> empty;
  mapper.integrate(Point(.05, .05, 1.0), ground, empty, ground);
  EXPECT_EQ(cost(mapper.project(), ground[0]), -1);
  mapper.integrate(Point(.05, .05, 1.0), ground, empty, ground);
  EXPECT_EQ(cost(mapper.project(), ground[0]), 0);
  EXPECT_EQ(cost(mapper.project(), Point(1.05, .05, 0)), -1);
}

TEST(Occupancy, HitWinsInSameFrameAndNoInputKeepsObstacle)
{
  Settings settings;
  OccupancyMapper mapper(settings);
  std::vector<Point> returns;
  returns.emplace_back(3.05, .05, 1.0);
  std::vector<Point> obstacle;
  obstacle.emplace_back(2.05, .05, 1.0);
  std::vector<Point> empty;
  mapper.integrate(Point(.05, .05, 1.0), returns, obstacle, empty);
  EXPECT_EQ(cost(mapper.project(), obstacle[0]), 100);
  mapper.integrate(Point(.05, .05, 1.0), empty, empty, empty);
  EXPECT_EQ(cost(mapper.project(), obstacle[0]), 100);
}

TEST(Occupancy, ActualFreeRaysCanRemoveOldHitWithGroundSupport)
{
  Settings settings;
  OccupancyMapper mapper(settings);
  std::vector<Point> obstacle;
  obstacle.emplace_back(2.05, .05, 1.0);
  std::vector<Point> ground;
  ground.emplace_back(2.05, .05, 0.0);
  std::vector<Point> empty;
  mapper.integrate(Point(.05, .05, 1.0), obstacle, obstacle, ground);
  std::vector<Point> visible;
  visible.emplace_back(3.05, .05, 1.0);
  for (int i = 0; i < 6; ++i)
  {
    mapper.integrate(Point(.05, .05, 1.0), visible, empty, ground);
  }
  EXPECT_EQ(cost(mapper.project(), ground[0]), 0);
}

TEST(Occupancy, NegativeWorldCellsAndSaveDoNotChangeLiveMap)
{
  Settings settings;
  OccupancyMapper mapper(settings);
  std::vector<Point> obstacle;
  obstacle.emplace_back(-2.05, -.05, 1.0);
  std::vector<Point> empty;
  mapper.integrate(Point(-.05, -.05, 1.0), obstacle, obstacle, empty);
  const auto before = mapper.project();
  EXPECT_EQ(cost(before, obstacle[0]), 100);
  const auto directory = std::filesystem::temp_directory_path() / "lidar_mapping_unit_export";
  const auto prefix = (directory / "map").string();
  ASSERT_TRUE(mapper.save(prefix, "odom"));
  EXPECT_EQ(before.data, mapper.project().data);
  EXPECT_TRUE(std::filesystem::exists(prefix + ".yaml"));
  std::filesystem::remove_all(directory);
}

TEST(Occupancy, PrunedCoarseLeavesCoverTheirFullFootprint)
{
  Settings settings;
  settings.prune_interval = 1;
  settings.incremental_projection = true;
  OccupancyMapper mapper(settings);
  std::vector<Point> obstacle;
  for (int x = 0; x < 2; ++x)
  {
    for (int y = 0; y < 2; ++y)
    {
      for (int z = 0; z < 2; ++z)
      {
        obstacle.emplace_back(.05 + .1 * x, .05 + .1 * y, .05 + .1 * z);
      }
    }
  }
  std::vector<Point> empty;
  mapper.integrate(Point(.05, .05, 1.0), empty, obstacle, empty);
  const auto cached = mapper.project();
  EXPECT_EQ(cached.data, mapper.project_full().data);
  EXPECT_EQ(std::count(cached.data.begin(), cached.data.end(), 100), 4);
  EXPECT_LT(mapper.nodes(), 25u);
}

TEST(Occupancy, CachedColumnsKeepOtherHeightsAndClearAfterRealRays)
{
  Settings settings;
  settings.prune_interval = 1;
  settings.incremental_projection = true;
  OccupancyMapper mapper(settings);
  std::vector<Point> obstacle;
  obstacle.emplace_back(2.05, .05, .5);
  obstacle.emplace_back(2.05, .05, 1.0);
  std::vector<Point> ground;
  ground.emplace_back(2.05, .05, 0.0);
  std::vector<Point> empty;
  mapper.integrate(Point(.05, .05, 1.0), empty, obstacle, ground);
  std::vector<Point> returns;
  returns.emplace_back(3.05, .05, 1.0);
  for (int i = 0; i < 6; ++i)
  {
    mapper.integrate(Point(.05, .05, 1.0), returns, empty, ground);
    EXPECT_EQ(mapper.project().data, mapper.project_full().data);
  }
  EXPECT_EQ(cost(mapper.project(), obstacle[0]), 100);
  returns[0] = Point(3.05, .05, .5);
  for (int i = 0; i < 6; ++i)
  {
    mapper.integrate(Point(.05, .05, .5), returns, empty, ground);
    EXPECT_EQ(mapper.project().data, mapper.project_full().data);
  }
  EXPECT_EQ(cost(mapper.project(), obstacle[0]), 0);
}

TEST(Occupancy, NativePathUpdatesMatchLegacyForMixedHitsAndClearing)
{
  Settings settings;
  OccupancyMapper baseline(settings);
  settings.incremental_projection = true;
  settings.incremental_inner_updates = true;
  OccupancyMapper candidate(settings);
  uint32_t state = 17;
  const auto coordinate = [&state]()
  {
    state = state * 1664525u + 1013904223u;
    return static_cast<float>(static_cast<int>((state >> 16) % 40) - 20) * .1F + .05F;
  };
  for (int frame = 0; frame < 40; ++frame)
  {
    std::vector<Point> returns;
    std::vector<Point> obstacles;
    std::vector<Point> ground;
    for (int i = 0; i < 80; ++i)
    {
      const float x = coordinate();
      const float y = coordinate();
      returns.emplace_back(x, y, i % 3 == 0 ? .05F : 1.05F);
      if (i % 3 == 0)
      {
        ground.push_back(returns.back());
      }
      else if (i % 2 == 0)
      {
        obstacles.push_back(returns.back());
      }
    }
    const Point origin(.05, .05, 1.05);
    baseline.integrate(origin, returns, obstacles, ground);
    candidate.integrate(origin, returns, obstacles, ground);
    const auto expected = baseline.project();
    const auto actual = candidate.project();
    EXPECT_EQ(expected.width, actual.width);
    EXPECT_EQ(expected.height, actual.height);
    EXPECT_DOUBLE_EQ(expected.origin_x, actual.origin_x);
    EXPECT_DOUBLE_EQ(expected.origin_y, actual.origin_y);
    EXPECT_EQ(expected.data, actual.data);
    EXPECT_EQ(actual.data, candidate.project_full().data);
  }
}
}
