#include "mid360_monitor/scan_timing.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>
#include <utility>

namespace mid360_monitor
{

ScanTimingAnalyzer::ScanTimingAnalyzer(int64_t pending_lidar_advance_ns)
: pending_lidar_advance_ns_(pending_lidar_advance_ns)
{
  if (pending_lidar_advance_ns_ <= 0)
  {
    throw std::invalid_argument("pending LiDAR advance must be positive");
  }
}

void ScanTimingAnalyzer::addImuStamp(int64_t stamp_ns)
{
  const auto position = std::lower_bound(
    imu_stamps_.begin(), imu_stamps_.end(), stamp_ns);
  if (position == imu_stamps_.end() || *position != stamp_ns)
  {
    imu_stamps_.insert(position, stamp_ns);
  }

  if (!latest_imu_ns_.has_value() || stamp_ns > *latest_imu_ns_)
  {
    latest_imu_ns_ = stamp_ns;
  }

  finalizeReadyScans();
  pruneImuHistory();
}

void ScanTimingAnalyzer::addScan(const ScanWindow & scan)
{
  if (scan.raw_end_ns < scan.begin_ns)
  {
    throw std::invalid_argument("scan end precedes scan begin");
  }

  const auto position = std::upper_bound(
    pending_scans_.begin(), pending_scans_.end(), scan.raw_end_ns,
    [](int64_t end_ns, const ScanWindow & pending)
    {
      return end_ns < pending.raw_end_ns;
    });
  pending_scans_.insert(position, scan);

  if (!latest_lidar_begin_ns_.has_value() || scan.begin_ns > *latest_lidar_begin_ns_)
  {
    latest_lidar_begin_ns_ = scan.begin_ns;
  }

  finalizeReadyScans();
  pruneImuHistory();
}

std::vector<ScanCoverage> ScanTimingAnalyzer::takeCompleted()
{
  std::vector<ScanCoverage> result = std::move(completed_);
  completed_.clear();
  return result;
}

std::size_t ScanTimingAnalyzer::pendingCount() const
{
  return pending_scans_.size();
}

ScanCoverage ScanTimingAnalyzer::evaluate(const ScanWindow & scan, bool timed_out) const
{
  ScanCoverage result;
  result.scan = scan;
  result.timed_out_waiting_for_imu = timed_out;

  const auto first_inside = std::lower_bound(
    imu_stamps_.begin(), imu_stamps_.end(), scan.begin_ns);
  const auto after_inside = std::upper_bound(
    imu_stamps_.begin(), imu_stamps_.end(), scan.raw_end_ns);
  result.imu_count = static_cast<std::size_t>(
    std::distance(first_inside, after_inside));

  const auto after_start = std::upper_bound(
    imu_stamps_.begin(), imu_stamps_.end(), scan.begin_ns);
  auto interval_begin = first_inside;
  if (after_start != imu_stamps_.begin())
  {
    interval_begin = std::prev(after_start);
    result.start_gap_ns = scan.begin_ns - *interval_begin;
  }

  const auto first_after_end = std::lower_bound(
    imu_stamps_.begin(), imu_stamps_.end(), scan.raw_end_ns);
  auto interval_end = after_inside;
  if (first_after_end != imu_stamps_.end())
  {
    result.end_gap_ns = *first_after_end - scan.raw_end_ns;
    interval_end = std::next(first_after_end);
  }

  if (interval_begin != imu_stamps_.end() && interval_begin != interval_end)
  {
    for (auto current = std::next(interval_begin); current != interval_end; ++current)
    {
      const int64_t interval_ns = *current - *std::prev(current);
      if (!result.max_imu_interval_ns.has_value() ||
        interval_ns > *result.max_imu_interval_ns)
      {
        result.max_imu_interval_ns = interval_ns;
      }
    }
  }

  return result;
}

void ScanTimingAnalyzer::finalizeReadyScans()
{
  while (!pending_scans_.empty())
  {
    const ScanWindow & scan = pending_scans_.front();
    const bool has_imu_after_end =
      latest_imu_ns_.has_value() && *latest_imu_ns_ >= scan.raw_end_ns;
    const bool lidar_advanced_without_imu =
      latest_lidar_begin_ns_.has_value() &&
      *latest_lidar_begin_ns_ >= scan.raw_end_ns + pending_lidar_advance_ns_;

    if (!has_imu_after_end && !lidar_advanced_without_imu)
    {
      break;
    }

    completed_.push_back(evaluate(scan, !has_imu_after_end));
    pending_scans_.pop_front();
  }
}

void ScanTimingAnalyzer::pruneImuHistory()
{
  if (!latest_imu_ns_.has_value())
  {
    return;
  }

  int64_t cutoff_ns = *latest_imu_ns_ - 2000000000;
  if (!pending_scans_.empty())
  {
    const auto first_pending = std::min_element(
      pending_scans_.begin(), pending_scans_.end(),
      [](const ScanWindow & left, const ScanWindow & right)
      {
        return left.begin_ns < right.begin_ns;
      });
    cutoff_ns = std::min(cutoff_ns, first_pending->begin_ns);
  }

  while (imu_stamps_.size() > 1 && imu_stamps_[1] < cutoff_ns)
  {
    imu_stamps_.pop_front();
  }
}

}  // namespace mid360_monitor
