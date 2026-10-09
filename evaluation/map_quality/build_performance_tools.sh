#!/usr/bin/env bash
set -euo pipefail
PERF_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PERF_OUT="${1:?Usage: build_performance_tools.sh output_directory baseline_source_directory}"
PERF_BASE="${2:?Baseline source directory is required}"
PERF_PKG="$PERF_WS/src/lidar_perception"
mkdir -p "$PERF_OUT"
g++ -O3 -std=c++17 "$PERF_WS/evaluation/map_quality/benchmark_mapping.cpp" \
  "$PERF_WS/src/lidar_mapping/src/occupancy_mapper.cpp" -I"$PERF_WS/src/lidar_mapping/include" \
  -loctomap -loctomath -o "$PERF_OUT/benchmark_mapping"
# 两个旧实现直接链接，其他未改模块使用当前核心；不覆盖生产源码
for PERF_PROFILE in baseline optimized; do
  PERF_EXTRA=()
  if [[ "$PERF_PROFILE" == baseline ]]; then
    PERF_EXTRA=("$PERF_BASE/cloud_pipeline_instrumented.cpp" "$PERF_BASE/cluster_extractor.cpp")
  fi
  g++ -O3 -std=c++17 "$PERF_WS/evaluation/lidar_navigation/benchmark.cpp" "${PERF_EXTRA[@]}" \
    -I"$PERF_PKG/include" -I"$PERF_PKG/vendor/patchworkpp/cpp/patchworkpp/include" \
    -I"$PERF_PKG/vendor/patchworkpp/cpp/common/include" \
    $(pkg-config --cflags pcl_common pcl_filters pcl_segmentation) \
    "$PERF_WS/build/lidar_perception/libperception_core.a" \
    "$PERF_WS/build/lidar_perception/libpatchwork_core.a" \
    $(pkg-config --libs pcl_common pcl_filters pcl_segmentation) -o "$PERF_OUT/benchmark_$PERF_PROFILE"
done
