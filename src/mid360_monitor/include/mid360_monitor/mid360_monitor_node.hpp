#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mid360_monitor/scan_timing.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace mid360_monitor
{

struct StreamStats
{
  void recordSample(
    int64_t stamp_ns,
    std::chrono::steady_clock::time_point arrival_time);
  void resetWindow();
  std::optional<double> stampHz() const;
  std::optional<double> receiveHz() const;

  uint64_t received = 0;
  uint64_t valid_intervals = 0;
  uint64_t non_monotonic_intervals = 0;
  int64_t interval_sum_ns = 0;
  int64_t max_interval_ns = 0;
  std::optional<int64_t> latest_stamp_ns;
  std::optional<int64_t> latest_dt_ns;
  std::optional<int64_t> previous_stamp_ns;
  std::optional<std::chrono::steady_clock::time_point> first_arrival_time;
  std::optional<std::chrono::steady_clock::time_point> latest_arrival_time;
};

struct PointCountStats
{
  void recordFrame(uint32_t point_num, std::size_t points_size, uint64_t timebase_ns);
  void resetWindow();

  uint64_t count = 0;
  uint64_t point_sum = 0;
  uint64_t size_mismatch_count = 0;
  uint32_t point_min = 0;
  uint32_t point_max = 0;
  uint32_t latest_point_num = 0;
  std::size_t latest_points_size = 0;
  uint64_t latest_timebase_ns = 0;
};

struct ScanSummaryStats
{
  void recordCoverage(
    const ScanCoverage & coverage,
    int64_t imu_gap_warn_ns,
    int64_t scan_duration_min_ns,
    int64_t scan_duration_max_ns);
  void recordInvalidScan(int64_t header_stamp_ns);
  void resetWindow();

  uint64_t checked = 0;
  uint64_t complete = 0;
  uint64_t warning_frames = 0;
  uint64_t notable_frames = 0;
  uint64_t start_unknown = 0;
  uint64_t missing_end = 0;
  uint64_t no_imu = 0;
  uint64_t imu_gap_warn = 0;
  uint64_t duration_warn = 0;
  uint64_t timebase_mismatch = 0;
  uint64_t invalid_scan = 0;
  uint64_t imu_count_sum = 0;
  uint64_t duration_sum_ns = 0;
  std::size_t imu_count_min = 0;
  std::size_t imu_count_max = 0;
  int64_t duration_min_ns = 0;
  int64_t duration_max_ns = 0;
  std::vector<std::string> warning_details;
};

class Mid360MonitorNode : public rclcpp::Node
{
public:
  Mid360MonitorNode();

private:
  void onLidar(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);
  void onImu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  std::optional<ScanWindow> extractScanWindow(
    const livox_ros_driver2::msg::CustomMsg & msg) const;
  void consumeCompletedScans();
  void reportStatus();

  StreamStats lidar_stats_;
  StreamStats imu_stats_;
  PointCountStats point_stats_;
  ScanSummaryStats scan_stats_;
  ScanTimingAnalyzer timing_analyzer_;
  int64_t imu_gap_warn_ns_ = 0;
  int64_t scan_duration_min_ns_ = 0;
  int64_t scan_duration_max_ns_ = 0;
  std::chrono::steady_clock::time_point last_report_time_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

}  // namespace mid360_monitor
