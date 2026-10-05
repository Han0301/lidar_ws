#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace mid360_monitor
{

struct ScanWindow
{
  int64_t begin_ns = 0;
  int64_t raw_end_ns = 0;
  int64_t header_timebase_delta_ns = 0;
  uint32_t min_offset_ns = 0;
  uint32_t max_offset_ns = 0;
};

struct ScanCoverage
{
  ScanWindow scan;
  std::size_t imu_count = 0;
  std::optional<int64_t> start_gap_ns;
  std::optional<int64_t> end_gap_ns;
  std::optional<int64_t> max_imu_interval_ns;
  bool timed_out_waiting_for_imu = false;
};

class ScanTimingAnalyzer
{
public:
  explicit ScanTimingAnalyzer(int64_t pending_lidar_advance_ns = 500000000);

  void addImuStamp(int64_t stamp_ns);
  void addScan(const ScanWindow & scan);
  std::vector<ScanCoverage> takeCompleted();
  std::size_t pendingCount() const;

private:
  ScanCoverage evaluate(const ScanWindow & scan, bool timed_out) const;
  void finalizeReadyScans();
  void pruneImuHistory();

  int64_t pending_lidar_advance_ns_;
  std::optional<int64_t> latest_imu_ns_;
  std::optional<int64_t> latest_lidar_begin_ns_;
  std::deque<int64_t> imu_stamps_;
  std::deque<ScanWindow> pending_scans_;
  std::vector<ScanCoverage> completed_;
};

}  // namespace mid360_monitor
