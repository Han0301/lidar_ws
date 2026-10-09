#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <map>
#include <limits>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_perception
{
using Point = pcl::PointXYZI;      // 感知点类型：三维坐标（m）和反射强度
using Cloud = pcl::PointCloud<Point>;      // 点云容器；输入在 body，处理结果在调平后的输出坐标系

// 集中保存距离、地面与聚类参数；默认值可由 ROS YAML 覆盖
struct Parameters
{
  double min_range = 0.45;      // 有效点到 body 原点的最小三维距离（m）
  double max_range = 15.0;      // 有效点到 body 原点的最大三维距离（m）
  double voxel_size = 0.10;      // 体素降采样的格子边长（m）
  double reference_voxel_size = 0.10;      // 参考平面的独立采样分辨率（m），默认与原分类采样一致
  double ground_distance = 0.08;      // 平面拟合及近场地面判断的距离阈值（m）
  double ground_guard_distance = 0.16;      // 局部地面到参考平面允许的最大偏差（m）
  double obstacle_min_height = 0.12;      // 保留障碍点的最小离地高度（m）
  double obstacle_max_height = 2.50;      // 保留障碍点的最大离地高度（m）
  double cluster_tolerance = 0.30;      // 欧氏聚类相邻点的连接距离（m）
  int cluster_min_points = 6;      // 输出一个几何簇所需的最少点数；不影响障碍点云发布
  int cluster_max_points = 30000;      // 一个几何簇允许的最多点数
  std::string ground_method = "patchwork";      // 地面方法：patchwork 使用局部分割，plane 使用参考平面
};

// 描述一个空间排除盒，所有包含关系都在盒局部坐标中计算
struct ExclusionBox
{
  Eigen::Affine3f box_from_body = Eigen::Affine3f::Identity();      // 将 body 点变到排除盒局部坐标的变换
  Eigen::Vector3f half_size = Eigen::Vector3f::Zero();      // 排除盒沿局部 X/Y/Z 的半边长（m）
  std::string id;      // 区域标签，仅用于描述来源，不代表自动确认的人体身份
};

// 描述一个障碍几何簇的包围框，不赋予物体语义类别
struct Object
{
  Eigen::Vector3f center = Eigen::Vector3f::Zero();      // 几何包围框中心，位于调平后的输出坐标系（m）
  Eigen::Vector3f size = Eigen::Vector3f::Zero();      // 轴对齐包围框沿 X/Y/Z 的全边长（m）
  std::size_t points = 0;      // 组成该几何簇的点数
};

// 汇总一帧处理结果、参考地面可信状态与诊断统计
struct Result
{
  Cloud::Ptr ground = std::make_shared<Cloud>();      // 分割出的地面点，不等同于完整的可通行判断
  Cloud::Ptr obstacles = std::make_shared<Cloud>();      // 满足离地高度限制的非地面点，供地图标记
  Cloud::Ptr clearing = std::make_shared<Cloud>();      // 真实测量终点，保留排除区域内的回波，供射线清除
  Cloud::Ptr removed = std::make_shared<Cloud>();      // 按指定区域排除的点，仅用于诊断和可视化
  std::vector<Object> objects;      // 当前帧几何簇及包围框，没有类别识别或跨帧跟踪
  std::size_t input_points = 0;      // 原始输入点数
  std::size_t valid_points = 0;      // 通过有限值和距离筛选的点数
  std::size_t operator_points = 0;      // 落入排除区域的点数，并非人体识别数量
  double sensor_height = 0.0;      // 输出坐标系原点到参考地面的垂直高度估计（m）
  bool ground_reference_valid = false;      // 本帧是否成功取得可信参考平面，控制地图输入发布
  std::size_t reference_candidates = 0;      // 近场低处参考候选点数
  std::size_t reference_inliers = 0;      // 参考拟合的实际内点数
  std::string reference_status = "EMPTY";      // 地面参考失败原因或 OK
  double processing_ms = 0.0;      // 完整感知流水线的单帧墙钟耗时（ms）
  std::map<std::string, double> stage_ms;      // 独立阶段墙钟耗时，不含节点转换与发布
};
}
