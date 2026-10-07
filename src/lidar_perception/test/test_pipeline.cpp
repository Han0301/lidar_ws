#include "lidar_perception/cloud_pipeline.hpp"
#include <gtest/gtest.h>

namespace
{
using namespace lidar_perception;
void point(Cloud & cloud, float x, float y, float z)
{
  Point p;
  p.x = x; p.y = y; p.z = z; p.intensity = 30;
  cloud.push_back(p);
}
Cloud scene()
{
  Cloud cloud;
  for (float x = -5; x <= 5; x += .12F)
  {
    for (float y = -5; y <= 5; y += .12F)
    {
      point(cloud, x, y, -1.3F);
    }
  }
  // A 16 cm obstacle must survive ground removal, regardless of cluster size.
  for (float x = 2; x <= 2.5F; x += .04F)
  {
    for (float y = .5F; y <= 1; y += .04F)
    {
      point(cloud, x, y, -1.14F);
    }
  }
  // A larger high surface must not become the ground reference.
  for (float x = -3; x <= 3; x += .07F)
  {
    for (float y = -3; y <= 3; y += .07F)
    {
      point(cloud, x, y, -.2F);
    }
  }
  return cloud;
}
TEST(Perception, LowFloorWinsOverLargerTable)
{
  Parameters params;
  params.ground_method = "patchwork";
  CloudPipeline pipeline(params);
  auto result = pipeline.process(scene(), Eigen::Affine3f::Identity(), std::vector<ExclusionBox>());
  ASSERT_TRUE(result.ground_reference_valid);
  EXPECT_NEAR(result.sensor_height, 1.3, .04);
  std::size_t low_obstacle = 0;
  for (const auto & p : *result.obstacles)
  {
    if (p.x > 1.9 && p.x < 2.6 && p.y > .4 && p.y < 1.1 && p.z < -1.0)
    {
      ++low_obstacle;
    }
  }
  EXPECT_GT(low_obstacle, 10U);
  for (const auto & p : *result.ground)
  {
    EXPECT_LT(std::abs(p.z + 1.3F), .16F);
  }
}
TEST(Perception, TiltedScanGetsSameGroundHeight)
{
  Parameters params;
  CloudPipeline pipeline(params);
  auto cloud = scene();
  Eigen::Affine3f level = Eigen::Affine3f::Identity();
  level.linear() = Eigen::AngleAxisf(.6F, Eigen::Vector3f::UnitY()).toRotationMatrix();
  for (auto & p : cloud)
  {
    p.getVector3fMap() = level.inverse() * p.getVector3fMap();
  }
  auto result = pipeline.process(cloud, level, std::vector<ExclusionBox>());
  ASSERT_TRUE(result.ground_reference_valid);
  EXPECT_NEAR(result.sensor_height, 1.3, .04);
}
TEST(Perception, ExcludedReturnStillLimitsClearingRay)
{
  Parameters params;
  CloudPipeline pipeline(params);
  auto cloud = scene();
  point(cloud, .65F, .1F, 0);
  point(cloud, 1.2F, .1F, 0);
  ExclusionBox box;
  box.box_from_body.translation() = Eigen::Vector3f(-.65F, -.1F, 0);
  box.half_size = Eigen::Vector3f(.08F, .08F, .08F);
  auto result = pipeline.process(cloud, Eigen::Affine3f::Identity(), std::vector<ExclusionBox>(1, box));
  EXPECT_EQ(result.operator_points, 1U);
  EXPECT_EQ(result.removed->size(), 1U);
  bool endpoint = false;
  bool neighbor = false;
  for (const auto & p : *result.clearing)
  {
    endpoint = endpoint || ((p.getVector3fMap()-Eigen::Vector3f(.65F, .1F, 0)).norm() < .05F);
  }
  for (const auto & p : *result.obstacles)
  {
    neighbor = neighbor || ((p.getVector3fMap()-Eigen::Vector3f(1.2F, .1F, 0)).norm() < .05F);
  }
  EXPECT_TRUE(endpoint);
  EXPECT_TRUE(neighbor);
}
TEST(Perception, NoGroundEvidenceRetainsObstacle)
{
  Parameters params;
  CloudPipeline pipeline(params);
  Cloud cloud;
  point(cloud, 2, 0, .5);
  auto result = pipeline.process(cloud, Eigen::Affine3f::Identity(), std::vector<ExclusionBox>());
  EXPECT_FALSE(result.ground_reference_valid);
  EXPECT_TRUE(result.ground->empty());
  EXPECT_EQ(result.obstacles->size(), 1U);
}
TEST(Perception, FloorHeightIsMeasuredAndRaisedStepsRemainObstacles)
{
  std::vector<float> heights;
  heights.push_back(0.8F);
  heights.push_back(1.4F);
  heights.push_back(2.0F);
  std::vector<float> steps;
  steps.push_back(0.2F);
  steps.push_back(0.4F);
  steps.push_back(0.8F);
  for (const float height : heights)
  {
    for (const float step : steps)
    {
      Cloud cloud;
      for (float x = -5; x <= 5; x += .12F)
      {
        for (float y = -5; y <= 5; y += .12F)
        {
          point(cloud, x, y, -height);
        }
      }
      for (float x = 2; x <= 3; x += .08F)
      {
        for (float y = -1; y <= 1; y += .08F)
        {
          point(cloud, x, y, -height + step);
        }
      }
      Parameters parameters;
      CloudPipeline pipeline(parameters);
      auto result = pipeline.process(cloud, Eigen::Affine3f::Identity(), std::vector<ExclusionBox>());
      ASSERT_TRUE(result.ground_reference_valid);
      EXPECT_NEAR(result.sensor_height, height, .04);
      std::size_t raised = 0;
      for (const auto & p : *result.obstacles)
      {
        if (p.x >= 1.9 && p.x <= 3.1 && std::abs(p.z + height - step) < .05)
        {
          ++raised;
        }
      }
      EXPECT_GT(raised, 50U);
    }
  }
}

}
