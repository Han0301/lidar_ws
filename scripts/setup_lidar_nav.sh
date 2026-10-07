#!/usr/bin/env bash
# Source this file from any directory. Prefer locally extracted Nav2 if present.
LIDAR_NAV_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$LIDAR_NAV_WORKSPACE/install/local_setup.bash"
export LD_LIBRARY_PATH="$LIDAR_NAV_WORKSPACE/sdk_install/lib:${LD_LIBRARY_PATH:-}"
if [[ -d "$LIDAR_NAV_WORKSPACE/.deps/nav2/opt/ros/jazzy" ]]; then
  LIDAR_NAV_ROOT="$LIDAR_NAV_WORKSPACE/.deps/nav2"
  LIDAR_NAV_PREFIX="$LIDAR_NAV_ROOT/opt/ros/jazzy"
  export AMENT_PREFIX_PATH="$LIDAR_NAV_PREFIX:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$LIDAR_NAV_PREFIX:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$LIDAR_NAV_PREFIX/lib:$LIDAR_NAV_ROOT/usr/lib:$LIDAR_NAV_ROOT/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$LIDAR_NAV_PREFIX/lib/python3.12/site-packages:$LIDAR_NAV_ROOT/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
  export PATH="$LIDAR_NAV_PREFIX/bin:$PATH"
fi
