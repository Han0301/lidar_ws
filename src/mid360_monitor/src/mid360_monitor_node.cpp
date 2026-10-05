#include "mid360_monitor/mid360_monitor_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace mid360_monitor
{

namespace
{

constexpr std::size_t kWarningDetailLimit = 3;

std::string formatMilliseconds(const std::optional<int64_t> & nanoseconds)
{
  if (!nanoseconds.has_value())
  {
    return "n/a";
  }

  std::ostringstream output;
  output << std::fixed << std::setprecision(2) <<
    static_cast<double>(*nanoseconds) / 1e6;
  return output.str();
}

std::string formatRate(const std::optional<double> & rate)
{
  if (!rate.has_value())
  {
    return "n/a";
  }

  std::ostringstream output;
  output << std::fixed << std::setprecision(2) << *rate;
  return output.str();
}

void appendReason(std::string & reasons, const char * reason)
{
  if (!reasons.empty())
  {
    reasons += ",";
  }
  reasons += reason;
}

}  // namespace

void StreamStats::recordSample(
  int64_t stamp_ns,
  std::chrono::steady_clock::time_point arrival_time)
{
  if (!first_arrival_time.has_value())
  {
    first_arrival_time = arrival_time;
  }
  latest_arrival_time = arrival_time;
  ++received;
  latest_stamp_ns = stamp_ns;

  if (previous_stamp_ns.has_value())
  {
    const int64_t dt_ns = stamp_ns - *previous_stamp_ns;
    latest_dt_ns = dt_ns;
    if (dt_ns > 0)
    {
      ++valid_intervals;
      interval_sum_ns += dt_ns;
      if (dt_ns > max_interval_ns)
      {
        max_interval_ns = dt_ns;
      }
    }
    else
    {
      ++non_monotonic_intervals;
    }
  }

  previous_stamp_ns = stamp_ns;
}

void StreamStats::resetWindow()
{
  received = 0;
  valid_intervals = 0;
  non_monotonic_intervals = 0;
  interval_sum_ns = 0;
  max_interval_ns = 0;
  latest_stamp_ns.reset();
  latest_dt_ns.reset();
  first_arrival_time.reset();
  latest_arrival_time.reset();
}

std::optional<double> StreamStats::stampHz() const
{
  if (valid_intervals == 0 || interval_sum_ns <= 0)
  {
    return std::nullopt;
  }
  return static_cast<double>(valid_intervals) * 1e9 /
         static_cast<double>(interval_sum_ns);
}

std::optional<double> StreamStats::receiveHz() const
{
  if (received < 2 || !first_arrival_time.has_value() ||
    !latest_arrival_time.has_value())
  {
    return std::nullopt;
  }

  const double elapsed_s = std::chrono::duration<double>(
    *latest_arrival_time - *first_arrival_time).count();
  if (elapsed_s <= 0.0)
  {
    return std::nullopt;
  }
  return static_cast<double>(received - 1) / elapsed_s;
}

void PointCountStats::recordFrame(
  uint32_t point_num,
  std::size_t points_size,
  uint64_t timebase_ns)
{
  if (count == 0)
  {
    point_min = point_num;
    point_max = point_num;
  }
  else
  {
    if (point_num < point_min)
    {
      point_min = point_num;
    }
    if (point_num > point_max)
    {
      point_max = point_num;
    }
  }

  ++count;
  point_sum += point_num;
  latest_point_num = point_num;
  latest_points_size = points_size;
  latest_timebase_ns = timebase_ns;
  if (static_cast<std::size_t>(point_num) != points_size)
  {
    ++size_mismatch_count;
  }
}

void PointCountStats::resetWindow()
{
  count = 0;
  point_sum = 0;
  size_mismatch_count = 0;
  point_min = 0;
  point_max = 0;
  latest_point_num = 0;
  latest_points_size = 0;
  latest_timebase_ns = 0;
}

void ScanSummaryStats::recordCoverage(
  const ScanCoverage & coverage,
  int64_t imu_gap_warn_ns,
  int64_t scan_duration_min_ns,
  int64_t scan_duration_max_ns)
{
  ++checked;
  const int64_t duration_ns = coverage.scan.raw_end_ns - coverage.scan.begin_ns;
  if (checked == 1)
  {
    duration_min_ns = duration_ns;
    duration_max_ns = duration_ns;
    imu_count_min = coverage.imu_count;
    imu_count_max = coverage.imu_count;
  }
  else
  {
    duration_min_ns = std::min(duration_min_ns, duration_ns);
    duration_max_ns = std::max(duration_max_ns, duration_ns);
    imu_count_min = std::min(imu_count_min, coverage.imu_count);
    imu_count_max = std::max(imu_count_max, coverage.imu_count);
  }
  duration_sum_ns += static_cast<uint64_t>(duration_ns);
  imu_count_sum += coverage.imu_count;

  const bool unknown_start = !coverage.start_gap_ns.has_value();
  const bool no_end = !coverage.end_gap_ns.has_value();
  const bool empty_imu = coverage.imu_count == 0;
  const bool bad_imu_gap = coverage.max_imu_interval_ns.has_value() &&
    *coverage.max_imu_interval_ns > imu_gap_warn_ns;
  const bool bad_duration = duration_ns < scan_duration_min_ns ||
    duration_ns > scan_duration_max_ns;
  const bool bad_timebase = coverage.scan.header_timebase_delta_ns != 0;

  start_unknown += unknown_start;
  missing_end += no_end;
  no_imu += empty_imu;
  imu_gap_warn += bad_imu_gap;
  duration_warn += bad_duration;
  timebase_mismatch += bad_timebase;

  const bool warning = no_end || empty_imu || bad_imu_gap ||
    bad_duration || bad_timebase;
  if (warning)
  {
    ++warning_frames;
  }
  else if (!unknown_start)
  {
    ++complete;
  }

  if (warning || unknown_start)
  {
    ++notable_frames;
  }
  if ((warning || unknown_start) && warning_details.size() < kWarningDetailLimit)
  {
    std::string reasons;
    if (unknown_start)
    {
      appendReason(reasons, "START_UNKNOWN");
    }
    if (no_end)
    {
      appendReason(reasons, "NO_END_IMU");
    }
    if (empty_imu)
    {
      appendReason(reasons, "NO_IMU_INSIDE");
    }
    if (bad_imu_gap)
    {
      appendReason(reasons, "IMU_GAP");
    }
    if (bad_duration)
    {
      appendReason(reasons, "SCAN_DURATION");
    }
    if (bad_timebase)
    {
      appendReason(reasons, "TIMEBASE_MISMATCH");
    }

    std::ostringstream detail;
    detail << "  scan begin_ns=" << coverage.scan.begin_ns <<
      " raw_end_ns=" << coverage.scan.raw_end_ns <<
      " imu=" << coverage.imu_count <<
      " start_gap_ms=" << formatMilliseconds(coverage.start_gap_ns) <<
      " end_gap_ms=" << formatMilliseconds(coverage.end_gap_ns) <<
      " max_imu_dt_ms=" << formatMilliseconds(coverage.max_imu_interval_ns) <<
      " reason=" << reasons;
    warning_details.push_back(detail.str());
  }
}

void ScanSummaryStats::recordInvalidScan(int64_t header_stamp_ns)
{
  ++invalid_scan;
  ++warning_frames;
  ++notable_frames;
  if (warning_details.size() < kWarningDetailLimit)
  {
    warning_details.push_back(
      "  scan begin_ns=" + std::to_string(header_stamp_ns) + " reason=INVALID_SCAN");
  }
}

void ScanSummaryStats::resetWindow()
{
  ScanSummaryStats empty;
  *this = std::move(empty);
}

Mid360MonitorNode::Mid360MonitorNode()
: Node("mid360_monitor_node"),
  last_report_time_(std::chrono::steady_clock::now())
{
  const std::string lidar_topic =
    declare_parameter<std::string>("lidar_topic", "/livox/lidar");
  const std::string imu_topic =
    declare_parameter<std::string>("imu_topic", "/livox/imu");
  const double report_period_s =
    declare_parameter<double>("report_period_s", 5.0);
  const double imu_gap_warn_ms =
    declare_parameter<double>("imu_gap_warn_ms", 15.0);
  const double scan_duration_min_ms =
    declare_parameter<double>("scan_duration_min_ms", 50.0);
  const double scan_duration_max_ms =
    declare_parameter<double>("scan_duration_max_ms", 150.0);

  if (!std::isfinite(report_period_s) || report_period_s <= 0.0)
  {
    throw std::invalid_argument("report_period_s must be positive");
  }
  if (!std::isfinite(imu_gap_warn_ms) || imu_gap_warn_ms <= 0.0 ||
    !std::isfinite(scan_duration_min_ms) || scan_duration_min_ms <= 0.0 ||
    !std::isfinite(scan_duration_max_ms) ||
    scan_duration_max_ms <= scan_duration_min_ms)
  {
    throw std::invalid_argument("invalid timing warning thresholds");
  }

  imu_gap_warn_ns_ = static_cast<int64_t>(std::llround(imu_gap_warn_ms * 1e6));
  scan_duration_min_ns_ = static_cast<int64_t>(
    std::llround(scan_duration_min_ms * 1e6));
  scan_duration_max_ns_ = static_cast<int64_t>(
    std::llround(scan_duration_max_ms * 1e6));

  const auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile();
  const auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  lidar_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
    lidar_topic,
    lidar_qos,
    [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
    {
      onLidar(msg);
    });
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic,
    imu_qos,
    [this](sensor_msgs::msg::Imu::ConstSharedPtr msg)
    {
      onImu(msg);
    });

  report_timer_ = create_wall_timer(
    std::chrono::duration<double>(report_period_s),
    [this]()
    {
      reportStatus();
    });

  RCLCPP_INFO(
    get_logger(),
    "Monitoring lidar=%s and imu=%s; summary=%.1fs; IMU gap>%.1fms; "
    "raw scan duration outside [%.1f, %.1f]ms; raw_end=timebase+max(offset_time)",
    lidar_topic.c_str(), imu_topic.c_str(), report_period_s,
    imu_gap_warn_ms, scan_duration_min_ms, scan_duration_max_ms);
}

void Mid360MonitorNode::onLidar(
  livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg)
{
  const auto stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  lidar_stats_.recordSample(stamp_ns, std::chrono::steady_clock::now());
  point_stats_.recordFrame(msg->point_num, msg->points.size(), msg->timebase);

  const auto scan = extractScanWindow(*msg);
  if (scan.has_value())
  {
    timing_analyzer_.addScan(*scan);
    consumeCompletedScans();
  }
  else
  {
    scan_stats_.recordInvalidScan(stamp_ns);
  }
}

void Mid360MonitorNode::onImu(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const auto stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  imu_stats_.recordSample(stamp_ns, std::chrono::steady_clock::now());
  timing_analyzer_.addImuStamp(stamp_ns);
  consumeCompletedScans();
}

std::optional<ScanWindow> Mid360MonitorNode::extractScanWindow(
  const livox_ros_driver2::msg::CustomMsg & msg) const
{
  const int64_t header_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
  if (header_ns <= 0 || msg.timebase == 0 || msg.points.empty())
  {
    return std::nullopt;
  }

  uint32_t min_offset_ns = std::numeric_limits<uint32_t>::max();
  uint32_t max_offset_ns = 0;
  for (const auto & point : msg.points)
  {
    min_offset_ns = std::min(min_offset_ns, point.offset_time);
    max_offset_ns = std::max(max_offset_ns, point.offset_time);
  }

  if (msg.timebase > static_cast<uint64_t>(
    std::numeric_limits<int64_t>::max() - max_offset_ns))
  {
    return std::nullopt;
  }

  const int64_t timebase_ns = static_cast<int64_t>(msg.timebase);
  const int64_t raw_end_ns = timebase_ns + max_offset_ns;
  if (raw_end_ns < header_ns)
  {
    return std::nullopt;
  }

  ScanWindow scan;
  scan.begin_ns = header_ns;
  scan.raw_end_ns = raw_end_ns;
  scan.header_timebase_delta_ns = header_ns - timebase_ns;
  scan.min_offset_ns = min_offset_ns;
  scan.max_offset_ns = max_offset_ns;
  return scan;
}

void Mid360MonitorNode::consumeCompletedScans()
{
  for (const auto & coverage : timing_analyzer_.takeCompleted())
  {
    scan_stats_.recordCoverage(
      coverage, imu_gap_warn_ns_, scan_duration_min_ns_, scan_duration_max_ns_);
  }
}

void Mid360MonitorNode::reportStatus()
{
  const auto now = std::chrono::steady_clock::now();
  const double elapsed_s = std::chrono::duration<double>(
    now - last_report_time_).count();
  if (elapsed_s <= 0.0)
  {
    return;
  }

  const bool no_data = lidar_stats_.received == 0 || imu_stats_.received == 0;
  const bool warning = scan_stats_.warning_frames > 0 ||
    lidar_stats_.non_monotonic_intervals > 0 ||
    imu_stats_.non_monotonic_intervals > 0 ||
    point_stats_.size_mismatch_count > 0;
  const char * status = no_data ? "NO_DATA" :
    (warning ? "WARN" : (scan_stats_.start_unknown > 0 ? "CHECK" : "OK"));

  std::ostringstream output;
  output << std::fixed << std::setprecision(2);
  output << "[MID360 " << elapsed_s << "s] " << status <<
    " | scans=" << point_stats_.count <<
    " checked=" << scan_stats_.checked <<
    " pending=" << timing_analyzer_.pendingCount();

  output << "\n  LiDAR  rx=" << formatRate(lidar_stats_.receiveHz()) <<
    "Hz stamp=" << formatRate(lidar_stats_.stampHz()) << "Hz";
  if (lidar_stats_.latest_stamp_ns.has_value())
  {
    output << " last_stamp_s=" << std::setprecision(3) <<
      static_cast<double>(*lidar_stats_.latest_stamp_ns) / 1e9 <<
      std::setprecision(2);
  }
  if (point_stats_.count > 0)
  {
    const double point_mean = static_cast<double>(point_stats_.point_sum) /
      static_cast<double>(point_stats_.count);
    output << " points[min/avg/max]=" << point_stats_.point_min << "/" <<
      point_mean << "/" << point_stats_.point_max;
  }
  else
  {
    output << " points[min/avg/max]=n/a";
  }
  if (scan_stats_.checked > 0)
  {
    output << " raw_scan_ms[min/avg/max]=" <<
      static_cast<double>(scan_stats_.duration_min_ns) / 1e6 << "/" <<
      static_cast<double>(scan_stats_.duration_sum_ns) /
      static_cast<double>(scan_stats_.checked) / 1e6 << "/" <<
      static_cast<double>(scan_stats_.duration_max_ns) / 1e6;
  }
  else
  {
    output << " raw_scan_ms[min/avg/max]=n/a";
  }

  output << "\n  IMU    rx=" << formatRate(imu_stats_.receiveHz()) <<
    "Hz stamp=" << formatRate(imu_stats_.stampHz()) << "Hz";
  if (imu_stats_.latest_stamp_ns.has_value())
  {
    output << " last_stamp_s=" << std::setprecision(3) <<
      static_cast<double>(*imu_stats_.latest_stamp_ns) / 1e9 <<
      std::setprecision(2);
  }
  if (imu_stats_.valid_intervals > 0)
  {
    output << " dt_ms[avg/max]=" <<
      static_cast<double>(imu_stats_.interval_sum_ns) /
      static_cast<double>(imu_stats_.valid_intervals) / 1e6 << "/" <<
      static_cast<double>(imu_stats_.max_interval_ns) / 1e6;
  }
  else
  {
    output << " dt_ms[avg/max]=n/a";
  }

  output << "\n  COVER  complete=" << scan_stats_.complete <<
    " start_unknown=" << scan_stats_.start_unknown <<
    " missing_end=" << scan_stats_.missing_end <<
    " no_imu=" << scan_stats_.no_imu <<
    " gap_warn=" << scan_stats_.imu_gap_warn;
  if (scan_stats_.checked > 0)
  {
    output << " imu_per_scan[min/avg/max]=" << scan_stats_.imu_count_min << "/" <<
      static_cast<double>(scan_stats_.imu_count_sum) /
      static_cast<double>(scan_stats_.checked) << "/" <<
      scan_stats_.imu_count_max;
  }
  else
  {
    output << " imu_per_scan[min/avg/max]=n/a";
  }

  output << "\n  CHECK  lidar_nonmono=" <<
    lidar_stats_.non_monotonic_intervals <<
    " imu_nonmono=" << imu_stats_.non_monotonic_intervals <<
    " point_size_mismatch=" << point_stats_.size_mismatch_count <<
    " scan_duration_warn=" << scan_stats_.duration_warn <<
    " timebase_mismatch=" << scan_stats_.timebase_mismatch <<
    " invalid_scan=" << scan_stats_.invalid_scan;

  for (const auto & detail : scan_stats_.warning_details)
  {
    output << "\n" << detail;
  }
  if (scan_stats_.notable_frames > scan_stats_.warning_details.size())
  {
    output << "\n  ... " <<
      scan_stats_.notable_frames - scan_stats_.warning_details.size() <<
      " more notable scans";
  }

  if (warning || no_data)
  {
    RCLCPP_WARN(get_logger(), "%s", output.str().c_str());
  }
  else
  {
    RCLCPP_INFO(get_logger(), "%s", output.str().c_str());
  }

  lidar_stats_.resetWindow();
  imu_stats_.resetWindow();
  point_stats_.resetWindow();
  scan_stats_.resetWindow();
  last_report_time_ = now;
}

}  // namespace mid360_monitor
