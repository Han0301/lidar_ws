#include "lidar_mapping/occupancy_mapper.hpp"
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>

namespace lidar_mapping
{

// 使用 OctoMap 默认概率模型，不自行实现占用概率或射线遍历
OccupancyMapper::OccupancyMapper(const Settings & settings) : settings_(settings), tree_(settings.resolution > 0 ? settings.resolution : 0.1)
{
  if (settings.resolution <= 0 || settings.max_range <= 0 || settings.min_ground_observations < 1 ||

    settings.max_grid_cells == 0 || settings.prune_interval < 0 || !std::isfinite(settings.resolution) || !std::isfinite(settings.max_range))
  {
    throw std::invalid_argument("Invalid mapping settings");
  }
  tree_.enableChangeDetection(settings_.incremental_projection);
}

// 世界坐标负值也按 floor 归入固定格，不随机器人平移窗口
OccupancyMapper::Cell OccupancyMapper::cell(const octomap::point3d & point) const
{
  return std::make_pair(static_cast<int>(std::floor(point.x() / settings_.resolution)),
    static_cast<int>(std::floor(point.y() / settings_.resolution)));
}

// 超量程终点整体忽略，避免把未知远处伪装为完整观测
bool OccupancyMapper::in_range(const octomap::point3d & origin, const octomap::point3d & point) const
{
  return std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z()) &&
    (point - origin).norm() <= settings_.max_range;
}

// 同一帧命中优先于穿越射线，清除只使用真实测量终点
void OccupancyMapper::integrate

(
  const octomap::point3d & origin,
  const std::vector<octomap::point3d> & returns,
  const std::vector<octomap::point3d> & obstacles,
  const std::vector<octomap::point3d> & ground
)
{
  octomap::KeySet occupied;
  auto checkpoint = std::chrono::steady_clock::now();
  const auto stage = [this, &checkpoint](const std::string & name)
  {
    const auto end = std::chrono::steady_clock::now();
    timings_[name] = std::chrono::duration<double, std::milli>(end - checkpoint).count();
    checkpoint = end;
  };
  octomap::KeySet free;
  octomap::KeySet endpoints;
  octomap::OcTreeKey key;
  octomap::KeyRay ray;
  for (const auto & point : obstacles)
  {
    if (in_range(origin, point) && tree_.coordToKeyChecked(point, key))
    {
      occupied.insert(key);
    }
  }
  const std::size_t stride = settings_.max_ray_endpoints == 0 ? 1 :
    std::max<std::size_t>(1, (returns.size() + settings_.max_ray_endpoints - 1) / settings_.max_ray_endpoints);
  std::size_t rays = 0;
  for (std::size_t index = 0; index < returns.size(); index += stride)
  {
    const auto & point = returns[index];
    if (!in_range(origin, point) || !tree_.coordToKeyChecked(point, key) || !endpoints.insert(key).second)
    {
      continue;
    }
    if (tree_.computeRayKeys(origin, point, ray))
    {
      free.insert(ray.begin(), ray.end());
      ++rays;
    }
  }
  stage("ray_keys_ms");
  timings_["ray_endpoints"] = rays;
  timings_["return_points"] = returns.size();
  std::set<Cell> ground_cells;
  for (const auto & point : ground)
  {
    if (in_range(origin, point) && tree_.coordToKeyChecked(point, key))
    {
      ground_cells.insert(cell(point));
      free.insert(key);
    }
  }
  for (const auto & entry : ground_cells)
  {
    auto & observations = ground_observations_[entry];
    observations = std::min(observations + 1, settings_.min_ground_observations);
  }
  stage("ground_evidence_ms");
  for (const auto & entry : free)
  {
    if (occupied.find(entry) == occupied.end())
    {
      tree_.updateNode(entry, false, !settings_.incremental_inner_updates);
    }
  }
  for (const auto & entry : occupied)
  {
    tree_.updateNode(entry, true, !settings_.incremental_inner_updates);
  }
  stage("tree_update_ms");
  if (!settings_.incremental_inner_updates)
  {
    tree_.updateInnerOccupancy();
  }
  stage("inner_occupancy_ms");
  if (settings_.incremental_projection)
  {
    update_projection();
  }
  stage("projection_maintenance_ms");
  ++frames_;
  if (settings_.prune_interval > 0 && frames_ % settings_.prune_interval == 0)
  {
    tree_.prune();
  }
  stage("prune_ms");
}

// 原生变更键始终位于最细层，裁剪后查询仍返回覆盖它的叶节点
void OccupancyMapper::update_projection()
{
  for (auto it = tree_.changedKeysBegin(); it != tree_.changedKeysEnd(); ++it)
  {
    const auto & key = it->first;
    const auto * node = tree_.search(key);
    const bool occupied = node != nullptr && tree_.isNodeOccupied(node);
    const bool previous = occupied_keys_.find(key) != occupied_keys_.end();
    if (occupied == previous)
    {
      continue;
    }
    const auto column = cell(tree_.keyToCoord(key));
    if (occupied)
    {
      occupied_keys_.insert(key);
      ++occupied_columns_[column];
    }
    else
    {
      occupied_keys_.erase(key);
      auto entry = occupied_columns_.find(column);
      if (--entry->second == 0)
      {
        occupied_columns_.erase(entry);
      }
    }
  }
  tree_.resetChangeDetection();
}

Grid OccupancyMapper::project() const
{
  return settings_.incremental_projection ? grid(occupied_columns_) : project_full();
}

// 粗叶节点按键范围展开二维覆盖，不能仅投影其中心
Grid OccupancyMapper::project_full() const
{
  std::map<Cell, std::size_t> occupied;
  for (auto it = tree_.begin_leafs(); it != tree_.end_leafs(); ++it)
  {
    if (!tree_.isNodeOccupied(*it))
    {
      continue;
    }
    const auto span = 1u << (tree_.getTreeDepth() - it.getDepth());
    if (static_cast<uint64_t>(span) * span > settings_.max_grid_cells)
    {
      throw std::runtime_error("Occupied leaf exceeds configured grid bound");
    }
    const auto key = it.getIndexKey();
    for (unsigned x = 0; x < span; ++x)
    {
      for (unsigned y = 0; y < span; ++y)
      {
        octomap::OcTreeKey fine(key[0] + x, key[1] + y, key[2]);
        occupied[cell(tree_.keyToCoord(fine))] = 1;
      }
    }
  }
  return grid(occupied);
}

// 占用列优先；自由列须有重复地面终点，未观测列始终未知
Grid OccupancyMapper::grid(const std::map<Cell, std::size_t> & occupied) const
{
  Grid result;
  if (ground_observations_.empty() && occupied.empty())
  {
    return result;
  }
  const auto first = ground_observations_.empty() ? occupied.begin()->first : ground_observations_.begin()->first;
  int x0 = first.first;
  int x1 = x0;
  int y0 = first.second;
  int y1 = y0;
  const auto bounds = [&](const Cell & entry)
  {
    x0 = std::min(x0, entry.first);
    x1 = std::max(x1, entry.first);
    y0 = std::min(y0, entry.second);
    y1 = std::max(y1, entry.second);
  };
  for (const auto & entry : ground_observations_)
  {
    bounds(entry.first);
  }
  for (const auto & entry : occupied)
  {
    bounds(entry.first);
  }
  --x0; --y0; ++x1; ++y1;
  result.width = x1 - x0 + 1;
  result.height = y1 - y0 + 1;
  if (static_cast<std::size_t>(result.width) * result.height > settings_.max_grid_cells)
  {
    throw std::runtime_error("Global grid exceeds configured memory bound");
  }
  result.origin_x = x0 * settings_.resolution;
  result.origin_y = y0 * settings_.resolution;
  result.data.assign(static_cast<std::size_t>(result.width) * result.height, -1);
  for (const auto & entry : ground_observations_)
  {
    if (entry.second >= settings_.min_ground_observations)
    {
      result.data[(entry.first.second - y0) * result.width + entry.first.first - x0] = 0;
    }
  }
  for (const auto & entry : occupied)
  {
    result.data[(entry.first.second - y0) * result.width + entry.first.first - x0] = 100;
  }
  return result;
}

// 保存 Nav2 可读取的 PGM/YAML 与非破坏性 OctoMap 二进制副本
bool OccupancyMapper::save(const std::string & prefix, const std::string & frame) const
{
  const auto grid = project();
  if (grid.data.empty())
  {
    return false;
  }
  const std::filesystem::path path(prefix);
  if (!path.parent_path().empty())
  {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream image(prefix + ".pgm.tmp", std::ios::binary);
  image << "P5\n" << grid.width << ' ' << grid.height << "\n255\n";
  for (int row = grid.height - 1; row >= 0; --row)
  {
    for (int column = 0; column < grid.width; ++column)
    {
      const auto value = grid.data[row * grid.width + column];
      const unsigned char pixel = value < 0 ? 205 : (value == 100 ? 0 : 254);
      image.write(reinterpret_cast<const char *>(&pixel), 1);
    }
  }
  image.close();
  std::ofstream metadata(prefix + ".yaml.tmp");
  metadata << "image: " << path.filename().string() << ".pgm\nmode: trinary\nresolution: " << settings_.resolution
    << "\norigin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  metadata << "# frame: " << frame << "\n# Single LIO session; cross-session localization is required separately\n";
  metadata.close();
  if (!image || !metadata || !tree_.writeBinaryConst(prefix + ".bt.tmp"))
  {
    return false;
  }
  const std::vector<std::string> extensions =
  {
    ".pgm", ".yaml", ".bt"
  };
  for (const auto & extension : extensions)
  {
    std::filesystem::rename(prefix + extension + ".tmp", prefix + extension);
  }
  return true;
}

std::size_t OccupancyMapper::nodes() const
{
  return tree_.size();
}

const std::map<std::string, double> & OccupancyMapper::timings() const
{
  return timings_;
}
}
