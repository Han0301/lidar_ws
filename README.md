# lidar_ws：Livox MID-360 与 FAST-LIO2（ROS 2 Jazzy）

这个仓库保存本工作空间的自写 `mid360_monitor` 包，以及对 FAST_LIO 和 Livox ROS 驱动的本地适配补丁。上游项目按指定提交检出；bag、地图、构建产物和编辑器缓存不入库。

## 目录

```text
lidar_ws/
├── src/mid360_monitor/              # 点云与 IMU 的频率、时间戳监测
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
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/sdk_install/lib:${LD_LIBRARY_PATH:-}"
```

补丁里的地图输出路径是 `/home/h/lidar_ws/maps/fastlio_mid360.pcd`。如果工作空间不在 `/home/h/lidar_ws`，修改 `src/FAST_LIO/config/mid360.yaml` 的 `map_file_path`，并预先创建输出目录：

```bash
mkdir -p ~/lidar_ws/maps ~/lidar_ws/bags
```

## 快速启动

以下流程对应笔记《ld-1 常用命令》的 2.1–2.6。驱动配置当前使用电脑有线口 `enp4s0`、电脑 IP `192.168.1.50/24`、MID-360 IP `192.168.1.112`。

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

### 2.2 终端一：启动雷达驱动

先执行“每个新的终端”中的环境命令，再运行：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 2.3 终端二：启动 FAST-LIO2 建图

新终端加载环境后运行：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

启动后先静止几秒，再缓慢移动。RViz 的固定坐标系为 `camera_init`；查看 `/Laser_map` 地图和 `/Odometry` 轨迹。可以检查输入频率：

```bash
ros2 topic hz /livox/imu
ros2 topic hz /livox/lidar
```

### 2.4 终端三：录制离线 bag

新终端加载环境后运行：

```bash
MID360_BAG="$HOME/lidar_ws/bags/mid360_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -s mcap -o "$MID360_BAG" --topics /livox/lidar /livox/imu
```

停止录制时按 `Ctrl+C`。录制目录已被 Git 忽略。

### 2.5 可选可视化与里程计检查

建图启动命令默认会打开 RViz；需要单独打开时，新终端加载环境后运行：

```bash
rviz2 -d "$HOME/lidar_ws/src/FAST_LIO/rviz/fastlio.rviz"
```

查看位姿：

```bash
ros2 topic echo /Odometry --field pose.pose
```

查看自写监测节点的周期报告，可另开终端加载环境后运行：

```bash
ros2 launch mid360_monitor mid360_monitor.launch.py
```

### 2.6 离线播放与建图

先关闭实时驱动和在线建图。终端一加载环境后启动使用仿真时间的 FAST-LIO2：

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml use_sim_time:=true rviz:=false
```

终端二加载环境后，填入实际录制的 bag 目录并播放。先启动建图再播放，避免漏掉开头的数据：

```bash
MID360_BAG="$HOME/lidar_ws/bags/mid360_20261005_162040"
ros2 bag play "$MID360_BAG" --clock 100 -r 0.5
```

如果要单独查看 RViz，可使用 2.5 的命令。查询里程计消息：

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
