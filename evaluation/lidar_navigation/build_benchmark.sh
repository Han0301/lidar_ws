#!/usr/bin/env bash
set -euo pipefail
LIDAR_BENCH_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
LIDAR_BENCH_PACKAGE="$LIDAR_BENCH_ROOT/src/lidar_perception"
mkdir -p "$LIDAR_BENCH_ROOT/evaluation/lidar_navigation/results/tools"
g++ -O3 -std=c++17 "$LIDAR_BENCH_ROOT/evaluation/lidar_navigation/benchmark.cpp" \
  -I"$LIDAR_BENCH_PACKAGE/include" \
  -I"$LIDAR_BENCH_PACKAGE/vendor/patchworkpp/cpp/patchworkpp/include" \
  -I"$LIDAR_BENCH_PACKAGE/vendor/patchworkpp/cpp/common/include" \
  $(pkg-config --cflags pcl_common pcl_filters pcl_segmentation) \
  "$LIDAR_BENCH_ROOT/build/lidar_perception/libperception_core.a" \
  "$LIDAR_BENCH_ROOT/build/lidar_perception/libpatchwork_core.a" \
  $(pkg-config --libs pcl_common pcl_filters pcl_segmentation) \
  -o "$LIDAR_BENCH_ROOT/evaluation/lidar_navigation/results/tools/benchmark"
