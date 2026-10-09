#pragma once
#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include <octomap/OcTree.h>

namespace lidar_mapping
{

// OctoMap 负责三维占用概率；二维自由空间还必须有实际地面观测
struct Settings
{
  double resolution = 0.1;      // 三维体素与输出栅格分辨率（m）
  double max_range = 15.0;      // 真实回波与射线的最大距离（m）
  int min_ground_observations = 2;      // 同一地面格被不同帧观测的最低次数
  std::size_t max_grid_cells = 4000000;      // 全程栅格的内存上限
  bool incremental_projection = false;      // 按原生占用变化维护二维列计数
  bool incremental_inner_updates = false;      // 原生逐路径更新替代全树内部占用刷新
  int prune_interval = 0;      // 每隔指定积分帧执行原生无损裁剪，0 关闭
  std::size_t max_ray_endpoints = 0;      // 真实终点预算，0 保留全部，抽样不修改标记和地面证据
};

// 固定世界格上的二维投影，-1 未知、0 地面支持自由、100 占用
struct Grid
{
  int width = 0;
  int height = 0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<int8_t> data;
};

// 不依赖 ROS 的建图核心，接收逐帧真实射线及分类终点
class OccupancyMapper
{
public:
  explicit OccupancyMapper(const Settings & settings);
  void integrate
  (
    const octomap::point3d & origin,
    const std::vector<octomap::point3d> & returns,
    const std::vector<octomap::point3d> & obstacles,
    const std::vector<octomap::point3d> & ground
  );
  Grid project() const;
  Grid project_full() const;
  bool save(const std::string & prefix, const std::string & frame) const;
  std::size_t nodes() const;
  const std::map<std::string, double> & timings() const;

private:
  using Cell = std::pair<int, int>;
  Cell cell(const octomap::point3d & point) const;
  bool in_range(const octomap::point3d & origin, const octomap::point3d & point) const;
  Grid grid(const std::map<Cell, std::size_t> & occupied) const;
  void update_projection();
  Settings settings_;
  octomap::OcTree tree_;
  std::map<std::string, double> timings_;
  octomap::KeySet occupied_keys_;
  std::map<Cell, std::size_t> occupied_columns_;
  std::size_t frames_ = 0;
  std::map<Cell, int> ground_observations_;      // 每帧去重后的地面证据，不从射线或缺失点推断自由
};
}
