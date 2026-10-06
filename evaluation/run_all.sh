#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIDAR_WS="${LIDAR_WS:-$(cd "$SCRIPT_DIR/.." && pwd)}"
LIDAR_EVAL_RESULTS="${LIDAR_EVAL_RESULTS:-$SCRIPT_DIR/results}"
LIDAR_BAG_DIR="${LIDAR_BAG_DIR:-$LIDAR_WS/bags}"
export LIDAR_WS LIDAR_EVAL_RESULTS

if [[ ! -f "$LIDAR_WS/install/setup.bash" ]]; then
  echo "Missing $LIDAR_WS/install/setup.bash; build the workspace as described in README.md" >&2
  exit 1
fi
for bag in '室内静止' '室内闭环' '室外闭环'; do
  if [[ ! -f "$LIDAR_BAG_DIR/$bag/metadata.yaml" ]]; then
    echo "Missing bag: $LIDAR_BAG_DIR/$bag" >&2
    exit 1
  fi
done

source /opt/ros/jazzy/setup.bash
source "$LIDAR_WS/install/setup.bash"
mkdir -p "$LIDAR_EVAL_RESULTS"

cases=(
  '室内静止:indoor_static:181'
  '室内闭环:indoor_loop:182'
  '室外闭环:outdoor_loop:183'
)
for entry in "${cases[@]}"; do
  IFS=: read -r bag case domain <<< "$entry"
  echo "=== $bag: raw input analysis ==="
  /usr/bin/python3 "$SCRIPT_DIR/analyze_inputs.py" \
    "$LIDAR_BAG_DIR/$bag" "$LIDAR_EVAL_RESULTS/${case}_inputs.json"
  echo "=== $bag: FAST-LIO and monitor replay ==="
  /usr/bin/python3 "$SCRIPT_DIR/run_playback.py" \
    "$LIDAR_BAG_DIR/$bag" "$case" "$domain"
  /usr/bin/python3 "$SCRIPT_DIR/summarize_odometry.py" \
    "$LIDAR_EVAL_RESULTS/$case/odometry.csv" \
    "$LIDAR_EVAL_RESULTS/$case/odometry_summary.json"
done

/usr/bin/python3 "$SCRIPT_DIR/plot_results.py"
echo "Results: $LIDAR_EVAL_RESULTS"
