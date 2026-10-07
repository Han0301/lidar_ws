#!/usr/bin/env bash
set -euo pipefail
LIDAR_GROUND_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
LIDAR_GROUND_DEPS="$LIDAR_GROUND_ROOT/.deps/ground_consistency"
LIDAR_GROUND_SOURCE="$LIDAR_GROUND_DEPS/src/plugin"
LIDAR_GROUND_COMMIT=41cec620efba6c370dccfc59a6ec1134775ff48a
mkdir -p "$LIDAR_GROUND_DEPS/src"
if [[ ! -d "$LIDAR_GROUND_SOURCE/.git" ]]; then
  git clone https://github.com/dfki-ric/nav2_ground_consistency_costmap_plugin.git "$LIDAR_GROUND_SOURCE"
fi
if [[ "$(git -C "$LIDAR_GROUND_SOURCE" rev-parse HEAD)" != "$LIDAR_GROUND_COMMIT" ]]; then
  git -C "$LIDAR_GROUND_SOURCE" checkout "$LIDAR_GROUND_COMMIT"
fi
if git -C "$LIDAR_GROUND_SOURCE" apply --check "$LIDAR_GROUND_ROOT/patches/ground_consistency.patch" 2>/dev/null; then
  git -C "$LIDAR_GROUND_SOURCE" apply "$LIDAR_GROUND_ROOT/patches/ground_consistency.patch"
elif ! git -C "$LIDAR_GROUND_SOURCE" apply --reverse --check "$LIDAR_GROUND_ROOT/patches/ground_consistency.patch" 2>/dev/null; then
  echo "Dependency has unexpected edits; inspect it before applying the pinned patch." >&2
  exit 1
fi
set +u
source "$LIDAR_GROUND_ROOT/scripts/setup_lidar_nav.sh"
set -u
if [[ -d "$LIDAR_GROUND_ROOT/.deps/nav2/opt/ros/jazzy" ]]; then
  python3 "$LIDAR_GROUND_ROOT/scripts/relocate_local_nav2.py" "$LIDAR_GROUND_ROOT/.deps/nav2/opt/ros/jazzy"
fi
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build   --base-paths "$LIDAR_GROUND_SOURCE"   --build-base "$LIDAR_GROUND_DEPS/build"   --install-base "$LIDAR_GROUND_DEPS/install"   --merge-install --executor sequential   --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
