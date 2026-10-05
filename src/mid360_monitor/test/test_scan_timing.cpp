#include <cstdint>

#include "gtest/gtest.h"
#include "mid360_monitor/scan_timing.hpp"

namespace mid360_monitor
{
namespace
{

constexpr int64_t kMillisecond = 1000000;

ScanWindow makeScan(int64_t begin_ms, int64_t end_ms)
{
  ScanWindow scan;
  scan.begin_ns = begin_ms * kMillisecond;
  scan.raw_end_ns = end_ms * kMillisecond;
  return scan;
}

TEST(ScanTiming, CountsImuSamplesAndBracketsScan)
{
  ScanTimingAnalyzer analyzer;
  for (int64_t time_ms = -5; time_ms <= 105; time_ms += 5)
  {
    analyzer.addImuStamp(time_ms * kMillisecond);
  }
  analyzer.addScan(makeScan(0, 100));

  const auto completed = analyzer.takeCompleted();
  ASSERT_EQ(completed.size(), 1u);
  EXPECT_EQ(completed[0].imu_count, 21u);
  EXPECT_EQ(completed[0].start_gap_ns, 0);
  EXPECT_EQ(completed[0].end_gap_ns, 0);
  EXPECT_EQ(completed[0].max_imu_interval_ns, 5 * kMillisecond);
  EXPECT_FALSE(completed[0].timed_out_waiting_for_imu);
}

TEST(ScanTiming, ExposesLongImuInterval)
{
  ScanTimingAnalyzer analyzer;
  analyzer.addImuStamp(-5 * kMillisecond);
  analyzer.addImuStamp(0);
  analyzer.addImuStamp(5 * kMillisecond);
  analyzer.addImuStamp(25 * kMillisecond);
  for (int64_t time_ms = 30; time_ms <= 105; time_ms += 5)
  {
    analyzer.addImuStamp(time_ms * kMillisecond);
  }
  analyzer.addScan(makeScan(0, 100));

  const auto completed = analyzer.takeCompleted();
  ASSERT_EQ(completed.size(), 1u);
  EXPECT_EQ(completed[0].max_imu_interval_ns, 20 * kMillisecond);
}

TEST(ScanTiming, MarksUnknownBeginningWithoutCallingItMissingEnd)
{
  ScanTimingAnalyzer analyzer;
  for (int64_t time_ms = 5; time_ms <= 105; time_ms += 5)
  {
    analyzer.addImuStamp(time_ms * kMillisecond);
  }
  analyzer.addScan(makeScan(0, 100));

  const auto completed = analyzer.takeCompleted();
  ASSERT_EQ(completed.size(), 1u);
  EXPECT_FALSE(completed[0].start_gap_ns.has_value());
  EXPECT_TRUE(completed[0].end_gap_ns.has_value());
  EXPECT_EQ(completed[0].imu_count, 20u);
}

TEST(ScanTiming, KeepsBagTailPendingUntilLidarAdvances)
{
  ScanTimingAnalyzer analyzer;
  for (int64_t time_ms = 0; time_ms <= 95; time_ms += 5)
  {
    analyzer.addImuStamp(time_ms * kMillisecond);
  }
  analyzer.addScan(makeScan(0, 100));
  EXPECT_EQ(analyzer.pendingCount(), 1u);
  EXPECT_TRUE(analyzer.takeCompleted().empty());

  analyzer.addScan(makeScan(600, 700));
  const auto completed = analyzer.takeCompleted();
  ASSERT_EQ(completed.size(), 1u);
  EXPECT_TRUE(completed[0].timed_out_waiting_for_imu);
  EXPECT_FALSE(completed[0].end_gap_ns.has_value());
  EXPECT_EQ(analyzer.pendingCount(), 1u);
}

TEST(ScanTiming, SortsOutOfOrderImuStampsWithinPendingWindow)
{
  ScanTimingAnalyzer analyzer;
  analyzer.addScan(makeScan(0, 15));
  analyzer.addImuStamp(0);
  analyzer.addImuStamp(10 * kMillisecond);
  analyzer.addImuStamp(5 * kMillisecond);
  analyzer.addImuStamp(15 * kMillisecond);

  const auto completed = analyzer.takeCompleted();
  ASSERT_EQ(completed.size(), 1u);
  EXPECT_EQ(completed[0].imu_count, 4u);
  EXPECT_EQ(completed[0].max_imu_interval_ns, 5 * kMillisecond);
}

}  // namespace
}  // namespace mid360_monitor
