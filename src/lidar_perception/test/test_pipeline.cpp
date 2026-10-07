#include "lidar_perception/cloud_pipeline.hpp"
#include <gtest/gtest.h>

namespace
{
using namespace lidar_perception;
// 向合成场景添加指定坐标和固定强度的测试点
void point
(
  Cloud & cloud,
  float x,
  float y,
  float z
)
{
  Point p;      // 单个合成测量点，坐标单位 m
  p.x = x; p.y = y; p.z = z; p.intensity = 30;
  cloud.push_back(p);
}
// 构造地面、16 cm 低障碍与点数更多的高处桌面
Cloud scene()
{
  Cloud cloud;      // 当前测试构造的 body 坐标点云
  for (float x = -5; x <= 5; x += .12F)
  {
    for (float y = -5; y <= 5; y += .12F)
    {
      point(cloud, x, y, -1.3F);
    }
  }
  // 16 cm 低障碍必须在地面移除后保留，不依赖聚类是否生成包围框
  for (float x = 2; x <= 2.5F; x += .04F)
  {
    for (float y = .5F; y <= 1; y += .04F)
    {
      point(cloud, x, y, -1.14F);
    }
  }
  // 高处桌面虽有更多点，也不能取代低处的参考地面
  for (float x = -3; x <= 3; x += .07F)
  {
    for (float y = -3; y <= 3; y += .07F)
    {
      point(cloud, x, y, -.2F);
    }
  }
  return cloud;
}
// 验证参考地面选低处地面，并保留 16 cm 障碍而不误选大桌面
TEST(Perception, LowFloorWinsOverLargerTable)
{
  Parameters params;      // 本测试使用的流水线参数
  params.ground_method = "patchwork";
  CloudPipeline pipeline(params);      // 独立于 ROS 的单帧感知流水线
  auto result = pipeline.process(scene(), Eigen::Affine3f::Identity(), std::vector<ExclusionBox>());
  ASSERT_TRUE(result.ground_reference_valid);
  EXPECT_NEAR(result.sensor_height, 1.3, .04);
  std::size_t low_obstacle = 0;      // 分割后仍保留的低障碍点数
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
// 验证输入倾斜后经调平变换仍能恢复相同离地高度
TEST(Perception, TiltedScanGetsSameGroundHeight)
{
  Parameters params;      // 本测试使用的流水线参数
  CloudPipeline pipeline(params);      // 独立于 ROS 的单帧感知流水线
  auto cloud = scene();      // 具有地面、低障碍和大桌面的合成场景
  Eigen::Affine3f level = Eigen::Affine3f::Identity();      // 恢复调平坐标所用的 body→level 变换
  level.linear() = Eigen::AngleAxisf(.6F, Eigen::Vector3f::UnitY()).toRotationMatrix();
  for (auto & p : cloud)
  {
    p.getVector3fMap() = level.inverse() * p.getVector3fMap();
  }
  auto result = pipeline.process(cloud, level, std::vector<ExclusionBox>());
  ASSERT_TRUE(result.ground_reference_valid);
  EXPECT_NEAR(result.sensor_height, 1.3, .04);
}
// 验证空间排除只影响标记支路，清除射线仍止于原始遮挡回波
TEST(Perception, ExcludedReturnStillLimitsClearingRay)
{
  Parameters params;      // 本测试使用的流水线参数
  CloudPipeline pipeline(params);      // 独立于 ROS 的单帧感知流水线
  auto cloud = scene();      // 具有地面、低障碍和大桌面的合成场景
  point(cloud, .65F, .1F, 0);
  point(cloud, 1.2F, .1F, 0);
  ExclusionBox box;      // 模拟持机者空间排除区域
  box.box_from_body.translation() = Eigen::Vector3f(-.65F, -.1F, 0);
  box.half_size = Eigen::Vector3f(.08F, .08F, .08F);
  auto result = pipeline.process(cloud, Eigen::Affine3f::Identity(), std::vector<ExclusionBox>(1, box));
  EXPECT_EQ(result.operator_points, 1U);
  EXPECT_EQ(result.removed->size(), 1U);
  bool endpoint = false;      // 是否保留真实遮挡回波作为射线终点
  bool neighbor = false;      // 附近未被区域排除的障碍点是否保留
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
// 验证没有地面证据时保留非地面结果并将参考标为不可信
TEST(Perception, NoGroundEvidenceRetainsObstacle)
{
  Parameters params;      // 本测试使用的流水线参数
  CloudPipeline pipeline(params);      // 独立于 ROS 的单帧感知流水线
  Cloud cloud;      // 当前测试构造的 body 坐标点云
  point(cloud, 2, 0, .5);
  auto result = pipeline.process(cloud, Eigen::Affine3f::Identity(), std::vector<ExclusionBox>());
  EXPECT_FALSE(result.ground_reference_valid);
  EXPECT_TRUE(result.ground->empty());
  EXPECT_EQ(result.obstacles->size(), 1U);
}
// 验证不同离地高度都由点云实测，并保留不同高度的凸起
TEST(Perception, FloorHeightIsMeasuredAndRaisedStepsRemainObstacles)
{
  std::vector<float> heights;      // 待验证的不同传感器离地高度（m）
  heights.push_back(0.8F);
  heights.push_back(1.4F);
  heights.push_back(2.0F);
  std::vector<float> steps;      // 待验证的凸起相对地面高度（m）
  steps.push_back(0.2F);
  steps.push_back(0.4F);
  steps.push_back(0.8F);
  // 遍历离地高度与凸起高度组合，分别验证测高和障碍保留
  for (const float height : heights)
  {
    for (const float step : steps)
    {
      Cloud cloud;      // 当前测试构造的 body 坐标点云
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
      Parameters parameters;      // 每个高度组合使用默认感知参数
      CloudPipeline pipeline(parameters);
      auto result = pipeline.process(cloud, Eigen::Affine3f::Identity(), std::vector<ExclusionBox>());
      ASSERT_TRUE(result.ground_reference_valid);
      EXPECT_NEAR(result.sensor_height, height, .04);
      std::size_t raised = 0;      // 分割后仍保留的凸起点数
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
