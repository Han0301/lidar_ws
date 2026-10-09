# lidar_ws：MID-360 感知与占用建图（ROS 2 Jazzy）

基于 Livox MID-360 与 ROS 2，初步搭建点云/IMU → FAST-LIO2 里程计与去畸变 → TF2 坐标变换与重力对齐 → Patchwork++ 地面分割与 PCL 障碍提取 → Nav2 局部代价地图和 OctoMap 单会话占用建图的流水线。

仓库保存 `mid360_monitor`、`lidar_perception`、`lidar_mapping`、`lidar_nav2_bringup` 包、独立评测工具，以及 FAST_LIO 和 Livox ROS 驱动的本地适配补丁。上游项目按指定提交检出；bag、实测地图、报告、构建产物和编辑器缓存保存在本地。

## 目录

```text
lidar_ws/
├── src/lidar_perception/           # 地面、障碍、聚类与可选区域过滤
├── src/lidar_mapping/              # OctoMap 占用建图、二维投影与地图保存
├── src/lidar_nav2_bringup/          # 统一启动、重力参考与 Nav2 接入
├── evaluation/lidar_navigation/    # 户外录包与理想闭环评估
├── evaluation/map_quality/         # 地图质量、性能与固定输入回归评测
├── src/mid360_monitor/              # 点云与 IMU 的频率、时间戳监测
├── scripts/setup_lidar_nav.sh       # ROS、工作空间、SDK 与本地 Nav2 环境
├── patches/fast_lio.patch           # FAST_LIO 的 Jazzy/依赖/地图路径适配
├── patches/livox_ros_driver2.patch  # 本机与 MID-360 的网络地址
├── src/FAST_LIO/                   # 克隆后安装，上游仓库
├── src/livox_ros_driver2/          # 克隆后安装，上游仓库
└── src/Livox-SDK2/                 # 克隆后安装，上游仓库
```

## 首次部署：克隆依赖并构建

以下命令以 Ubuntu 24.04、ROS 2 Jazzy、仓库位置 `~/lidar_ws` 为例。需要已安装 ROS 2 Jazzy、`colcon`、`rosdep` 和 CMake。更换电脑时，先修改下文提到的网卡/IP、地图路径和 `patches/livox_ros_driver2.patch` 对应的驱动配置。

### 1. 克隆工作空间与固定版本的上游项目

```bash
git clone https://github.com/Han0301/lidar_ws.git ~/lidar_ws
cd ~/lidar_ws

git clone https://github.com/Livox-SDK/Livox-SDK2.git src/Livox-SDK2
git -C src/Livox-SDK2 checkout c0796f04c143143899c87a773d9f6b7136453c0b

git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2
git -C src/livox_ros_driver2 checkout 21445540f0d100dc86a7e6df312dd70bbdb4afdf

git clone --branch ROS2 https://github.com/hku-mars/FAST_LIO.git src/FAST_LIO
git -C src/FAST_LIO checkout a4743b095409588842a5b30ddfa27e29d2f99164
git -C src/FAST_LIO submodule update --init --recursive

git -C src/FAST_LIO apply ../../patches/fast_lio.patch
git -C src/livox_ros_driver2 apply ../../patches/livox_ros_driver2.patch
```

补丁已经包含当前工作空间对 `FAST_LIO/CMakeLists.txt`、`package.xml`、`config/mid360.yaml` 和驱动 `config/MID360_config.json` 的改动。`src/livox_ros_driver2/launch/` 中的本地文件与上游 `launch_ROS2/` 完全相同，无需单独提交或复制。

### 2. 构建 Livox SDK2

```bash
cd ~/lidar_ws
cmake -S src/Livox-SDK2 -B sdk_build -DCMAKE_INSTALL_PREFIX="$PWD/sdk_install"
cmake --build sdk_build -j"$(nproc)"
cmake --install sdk_build
```

### 3. 构建 ROS 2 工作空间

```bash
cd ~/lidar_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --merge-install --cmake-args \
  -DROS_EDITION=ROS2 \
  -DDISTRO_ROS=jazzy \
  -DLIVOX_LIDAR_SDK_LIBRARY="$PWD/sdk_install/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="$PWD/sdk_install/include"
```

每个新的终端先运行：

```bash
cd ~/lidar_ws
source scripts/setup_lidar_nav.sh
export ROS_DOMAIN_ID=0
```

该脚本加载 ROS 2 Jazzy、工作空间和 Livox SDK；存在 `.deps/nav2` 时同时加载本地 Nav2。所有参与同一链路的终端保持相同的 `ROS_DOMAIN_ID`。

补丁里的地图输出路径是 `/home/h/lidar_ws/maps/fastlio_mid360.pcd`。如果工作空间不在 `/home/h/lidar_ws`，修改 `src/FAST_LIO/config/mid360.yaml` 的 `map_file_path`，并预先创建输出目录：

```bash
mkdir -p ~/lidar_ws/maps ~/lidar_ws/bags
```

## 快速启动

以下流程对应笔记《ld-1 命令》的 2.1–2.6，使用当前主机的 `/home/h/lidar_ws` 路径；其他部署位置应相应修改路径。驱动配置当前使用电脑有线口 `enp4s0`、电脑 IP `192.168.1.50/24`、MID-360 IP `192.168.1.112`。

### 2.1 检查雷达供电和网络

接好网线与电源后运行：

```bash
ip -br link show dev enp4s0
ip -br -4 addr show dev enp4s0
ping -c 3 -I enp4s0 192.168.1.112
```

正常情况下，应看到 `LOWER_UP`、`192.168.1.50/24` 和 `0% packet loss`。网口有链路但没有 IP 时，可启用本机已有的连接配置：

```bash
nmcli connection up "Livox MID-360" ifname enp4s0
```

如果网卡或雷达地址不同，同时调整网络连接和 `src/livox_ros_driver2/config/MID360_config.json`。

### 2.2 每个终端先加载环境

实时雷达和录包回放均使用同一个系统入口，先执行：

```bash
cd /home/h/lidar_ws
source /home/h/lidar_ws/scripts/setup_lidar_nav.sh
export ROS_DOMAIN_ID=0
```

组件参数集中在 [system.yaml](src/lidar_nav2_bringup/config/system.yaml) 及其引用的 YAML 中。`source:=sensor` 使用真实雷达，`source:=bag` 使用录包时钟；入口同步设置各组件的 `use_sim_time`。

### 2.3 终端一：实时感知、建图与规划联调

```bash
ros2 launch lidar_nav2_bringup lidar.launch.py source:=sensor mode:=planning
```

自动启动 MID-360 驱动、FAST-LIO2、TF 桥接、感知、OctoMap 建图、Nav2 全局/局部代价地图、规划器及一个 RViz。启动后先静止几秒，再缓慢移动。

RViz 固定坐标系为 `odom`，显示 `/map`、地面、障碍、全局代价地图和估计轨迹。`/path` 是 FAST-LIO2 已估计轨迹，`/plan` 需要另行提交规划请求。此模式用于地图与规划联调，尚未包含真实底盘控制。

查看输入频率：

```bash
ros2 topic hz /livox/imu --wall-time
ros2 topic hz /livox/lidar --wall-time
```

### 2.4 终端二：录制离线 bag

新终端加载环境后运行：

```bash
MID360_BAG="/home/h/lidar_ws/bags/mid360_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -s mcap -o "$MID360_BAG" --topics /livox/lidar /livox/imu
```

停止录制时按 `Ctrl+C`。录制目录已被 Git 忽略。

### 2.5 仅建图、可视化与里程计检查

仅运行里程计、感知和占用建图时，用以下命令替换 2.3 的完整链路命令：

```bash
ros2 launch lidar_nav2_bringup lidar.launch.py source:=sensor mode:=mapping
```

`mapping` 启动 FAST-LIO2、感知、占用建图与 RViz；`planning` 另外启动 Nav2 代价地图和规划器。两种模式选一种运行；统一入口已包含对应组件。

默认打开 RViz，无需界面时追加 `rviz:=false`。布局为 [session_mapping.rviz](src/lidar_nav2_bringup/rviz/session_mapping.rviz)。地图保存前缀默认为 `/tmp/lidar_session_map`；长期保存时，在启动命令末尾追加 `output:=/home/h/lidar_ws/maps/session_map`，生成 PGM、YAML 和 OctoMap BT 文件。

查看位姿：

```bash
ros2 topic echo /Odometry --field pose.pose
```

查看自写监测节点的周期报告，可另开终端加载环境后运行：

```bash
ros2 launch mid360_monitor mid360_monitor.launch.py
```

### 2.6 录包回放下的感知、建图与规划

先结束实时系统。两个终端均执行 2.2 的环境命令，终端一启动系统：

```bash
ros2 launch lidar_nav2_bringup lidar.launch.py source:=bag mode:=planning
```

终端二按 1× 速度播放室外录包；也可替换为 2.4 新录制的目录：

```bash
ros2 bag play /home/h/lidar_ws/bags/室外闭环 --clock 100
```

仅做占用建图时，将 `mode:=planning` 改为 `mode:=mapping`；默认打开 RViz。查询里程计消息：

```bash
ros2 topic list
ros2 topic type /Odometry
ros2 interface show nav_msgs/msg/Odometry
ros2 topic echo /Odometry --once
ros2 topic echo /Odometry --field pose.pose.position
```

仓库中的命令与参数来自当前本机配置；构建通过不代表雷达连接、时间同步或建图精度已经在其他机器上验证。

## 对三个录包复跑测试

完成工作空间构建后，将 `室内静止`、`室内闭环`、`室外闭环` 三个 bag 目录放在 `bags/` 下，关闭正在运行的 FAST-LIO2，然后运行：

```bash
cd ~/lidar_ws
bash evaluation/run_all.sh
```

脚本会检查原始输入时间戳与 IMU 覆盖，再按原速回放并统计 FAST-LIO2 里程计。逐组命令、输出说明和已有结果见 [evaluation/README.md](evaluation/README.md) 与 [测试报告](evaluation/RESULTS_2026-10-06.md)。结果写入已忽略的 `evaluation/results/`，原始 bag 不会上传到仓库。

## 模块配置与评测

- [system.yaml](src/lidar_nav2_bringup/config/system.yaml)：统一入口的组件配置。
- [perception_dense_reference.yaml](src/lidar_perception/config/perception_dense_reference.yaml)：地面参考独立采样，分类体素与参考体素分别配置。
- [mapping.yaml](src/lidar_mapping/config/mapping.yaml)：0.10 m 全局占用图、原生逐路径更新、二维增量列缓存及扫描配对参数。
- [占用建图说明](src/lidar_mapping/README.md)：三态地图、真实观测清除、地图导出与接口。
- [感知与导航说明](evaluation/lidar_navigation/README.md)：组件调试入口、代码调用链、规划接口及仿真测试。
- [地图质量与性能评测](evaluation/map_quality/README.md)：录包采集、CPU/RSS、固定输入对照、地图一致性及生命周期退出回归。

更新源码后，在已加载环境的终端构建：

```bash
colcon build --merge-install --packages-select lidar_perception lidar_mapping lidar_nav2_bringup --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

项目目前完成录包回放下的感知、占用建图和规划接口联调；地图属于单会话 `odom` 坐标系，尚未接入回环优化、跨会话重定位或真实底盘导航。未观测空间保留为未知，地面门控有效率与估计轨迹首末间隔不作为分割准确率或绝对定位精度。实测报告与场地数据保存在本地。
