#include "lidar_mapping/occupancy_mapper.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/resource.h>

namespace fs = std::filesystem;

// 测量使用保存的真实世界系终点，算法不依赖评测文件
int main(int argc, char ** argv)
{
  if (argc != 9)
  {
    std::cerr << "inputs output cache prune_interval max_ray_endpoints frame_stride save_map incremental_inner\n";
    return 2;
  }
  lidar_mapping::Settings settings;
  settings.incremental_projection = std::stoi(argv[3]) != 0;
  settings.prune_interval = std::stoi(argv[4]);
  settings.max_ray_endpoints = std::stoul(argv[5]);
  settings.incremental_inner_updates = std::stoi(argv[8]) != 0;
  const auto frame_stride = std::stoul(argv[6]);
  if (frame_stride == 0)
  {
    return 2;
  }
  lidar_mapping::OccupancyMapper mapper(settings);
  const fs::path output(argv[2]);
  fs::create_directories(output);
  std::vector<fs::path> paths;
  for (const auto & item : fs::directory_iterator(argv[1]))
  {
    if (item.path().extension() == ".bin")
    {
      paths.push_back(item.path());
    }
  }
  std::sort(paths.begin(), paths.end());
  std::ofstream metrics(output / "metrics.csv");
  metrics << "stamp_ns,integrated,integration_ms,project_ms,nodes,width,height,origin_x,origin_y,ray_keys_ms,ground_evidence_ms,tree_update_ms,inner_occupancy_ms,projection_maintenance_ms,prune_ms,ray_endpoints,return_points\n";
  metrics << std::setprecision(12);
  const std::vector<std::string> timing_keys =
  {
    "ray_keys_ms", "ground_evidence_ms", "tree_update_ms", "inner_occupancy_ms",
    "projection_maintenance_ms", "prune_ms", "ray_endpoints", "return_points"
  };
  std::size_t index = 0;
  for (const auto & path : paths)
  {
    std::ifstream input(path, std::ios::binary);
    float origin[3];
    uint32_t sizes[3];
    input.read(reinterpret_cast<char *>(origin), sizeof(origin));
    input.read(reinterpret_cast<char *>(sizes), sizeof(sizes));
    std::vector<octomap::point3d> points[3];
    for (int group = 0; group < 3; ++group)
    {
      points[group].reserve(sizes[group]);
      for (uint32_t i = 0; i < sizes[group]; ++i)
      {
        float point[3];
        input.read(reinterpret_cast<char *>(point), sizeof(point));
        points[group].emplace_back(point[0], point[1], point[2]);
      }
    }
    if (!input)
    {
      return 3;
    }
    const bool integrated = index++ % frame_stride == 0;
    const auto began = std::chrono::steady_clock::now();
    if (integrated)
    {
      mapper.integrate(octomap::point3d(origin[0], origin[1], origin[2]), points[0], points[1], points[2]);
    }
    const auto projected = std::chrono::steady_clock::now();
    const auto grid = mapper.project();
    const auto end = std::chrono::steady_clock::now();
    std::ofstream raw(output / (path.stem().string() + ".grid"), std::ios::binary);
    raw.write(reinterpret_cast<const char *>(grid.data.data()), grid.data.size());
    metrics << path.stem().string() << ',' << integrated << ','
      << std::chrono::duration<double, std::milli>(projected - began).count() << ','
      << std::chrono::duration<double, std::milli>(end - projected).count() << ',' << mapper.nodes() << ','
      << grid.width << ',' << grid.height << ',' << grid.origin_x << ',' << grid.origin_y;
    for (const auto & key : timing_keys)
    {
      const auto & timings = mapper.timings();
      metrics << ',' << (integrated && timings.count(key) ? timings.at(key) : 0);
    }
    metrics << '\n';
  }
  rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  std::ofstream resources(output / "resources.json");
  resources << "{\"cpu_s\":" << usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
    usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6 << ",\"peak_rss_mib\":" << usage.ru_maxrss / 1024.0 << "}";
  if (std::stoi(argv[7]) != 0)
  {
    mapper.save((output / "global_map").string(), "odom");
  }
  return 0;
}
