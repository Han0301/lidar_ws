#include "lidar_perception/cloud_pipeline.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <iomanip>

namespace fs = std::filesystem;

void save(const fs::path & path, const lidar_perception::Cloud & cloud)
{
  std::ofstream out(path, std::ios::binary);
  for (const auto & p : cloud)
  {
    const float values[4] =
    {
      p.x, p.y, p.z, p.intensity
    };
    out.write(reinterpret_cast<const char *>(values), sizeof(values));
  }
}

int main(int argc, char ** argv)
{
  if (argc != 6 && argc != 7)
  {
    std::cerr << "input_directory output_directory voxel method mask [reference_voxel]\n";
    return 2;
  }
  fs::create_directories(argv[2]);
  lidar_perception::Parameters parameters;
  parameters.voxel_size = std::stod(argv[3]);
  parameters.ground_method = argv[4];
  parameters.reference_voxel_size = argc == 7 ? std::stod(argv[6]) : parameters.voxel_size;
  lidar_perception::CloudPipeline pipeline(parameters);
  std::vector<lidar_perception::ExclusionBox> boxes;
  if (std::string(argv[5]) == "candidate")
  {
    lidar_perception::ExclusionBox box;
    box.box_from_body.translation() = Eigen::Vector3f(0.625F, -0.275F, -0.325F);
    box.half_size = Eigen::Vector3f(.275F, .525F, .425F);
    boxes.push_back(box);
  }
  std::vector<fs::path> paths;
  for (const auto & item : fs::directory_iterator(argv[1]))
  {
    if (item.path().extension() == ".bin")
    {
      paths.push_back(item.path());
    }
  }
  std::sort(paths.begin(), paths.end());
  std::ofstream metrics(fs::path(argv[2]) / "metrics.csv");
  metrics << "frame,input,valid,removed,ground,obstacles,clusters,height,ground_valid,ms,reference_candidates,reference_inliers,reference_status\n";
  for (const auto & path : paths)
  {
    std::ifstream input(path, std::ios::binary);
    float values[9];
    input.read(reinterpret_cast<char *>(values), sizeof(values));
    Eigen::Affine3f level = Eigen::Affine3f::Identity();
    for (int row = 0; row < 3; ++row)
    {
      for (int col = 0; col < 3; ++col)
      {
        level.linear()(row, col) = values[row * 3 + col];
      }
    }
    lidar_perception::Cloud cloud;
    float point[4];
    while (input.read(reinterpret_cast<char *>(point), sizeof(point)))
    {
      lidar_perception::Point p;
      p.x = point[0]; p.y = point[1]; p.z = point[2]; p.intensity = point[3];
      cloud.push_back(p);
    }
    auto result = pipeline.process(cloud, level, boxes);
    metrics << path.stem().string() << ',' << result.input_points << ',' << result.valid_points << ',' << result.operator_points << ',' << result.ground->size() << ',' << result.obstacles->size() << ',' << result.objects.size() << ',' << result.sensor_height << ',' << result.ground_reference_valid << ',' << result.processing_ms << ',' << result.reference_candidates << ',' << result.reference_inliers << ',' << result.reference_status << '\n';
    save(fs::path(argv[2]) / (path.stem().string()+"_ground.bin"), *result.ground);
    save(fs::path(argv[2]) / (path.stem().string()+"_obstacles.bin"), *result.obstacles);
    save(fs::path(argv[2]) / (path.stem().string()+"_removed.bin"), *result.removed);
    save(fs::path(argv[2]) / (path.stem().string()+"_clearing.bin"), *result.clearing);
    std::ofstream objects(fs::path(argv[2]) / (path.stem().string() + "_objects.csv"));
    objects << std::setprecision(9);
    for (const auto & object : result.objects)
    {
      objects << object.center.x() << ',' << object.center.y() << ',' << object.center.z() << ','
        << object.size.x() << ',' << object.size.y() << ',' << object.size.z() << ',' << object.points << '\n';
    }
  }
}
