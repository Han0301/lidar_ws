# MID-360 录包测试复现

这组脚本读取 `bags/室内静止`、`bags/室内闭环`、`bags/室外闭环`，先检查原始点云与 IMU 消息，再以原速回放到当前已构建的 FAST-LIO2 和 `mid360_monitor`。每个 bag 使用独立的 ROS Domain；测试用的 FAST-LIO 配置副本关闭 PCD 保存，不改原始参数文件。脚本会备份并恢复 `src/FAST_LIO/Log` 中的原有文件。

## 一条命令运行全部测试

先按仓库根目录 README 构建工作空间，确认三个 bag 在上述目录，并具备 `ros-jazzy-rosbag2-py`、`ros-jazzy-rosbag2-storage-mcap` 和系统 Python 的 `python3-matplotlib`。当前测试机已安装这些依赖；其他 Jazzy 机器如缺少，可运行：

```bash
sudo apt install ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap python3-matplotlib
```

关闭正在运行的 FAST-LIO2，然后执行：

```bash
cd ~/lidar_ws
bash evaluation/run_all.sh
```

默认结果写入 `evaluation/results/`（Git 已忽略）。若 bag 存在其他位置，先设置 `LIDAR_BAG_DIR`；若工作空间或输出位置不同，可设置 `LIDAR_WS`、`LIDAR_EVAL_RESULTS`。

## 单独复跑一组

下面以“室内闭环”为例，使用 ROS 2 Jazzy 的系统 Python，避免 Conda 环境覆盖 ROS 模块：

```bash
cd ~/lidar_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LIDAR_WS="$PWD"
export LIDAR_EVAL_RESULTS="$PWD/evaluation/results"
mkdir -p "$LIDAR_EVAL_RESULTS"

/usr/bin/python3 evaluation/analyze_inputs.py \
  bags/室内闭环 "$LIDAR_EVAL_RESULTS/indoor_loop_inputs.json"
/usr/bin/python3 evaluation/run_playback.py \
  bags/室内闭环 indoor_loop 182
/usr/bin/python3 evaluation/summarize_odometry.py \
  "$LIDAR_EVAL_RESULTS/indoor_loop/odometry.csv" \
  "$LIDAR_EVAL_RESULTS/indoor_loop/odometry_summary.json"
```

把 bag 名、输出名和 Domain ID 分别换成 `室内静止 / indoor_static / 181` 或 `室外闭环 / outdoor_loop / 183`，即可运行另外两组。已有 ROS 任务占用这些 Domain 时，换一个未使用的 Domain ID。

## 测试口径

- 原始输入检查直接按 MCAP 中的消息读取，不受回放进程调度影响；检查点云与 IMU 的 `header.stamp` 单调性、相邻间隔、点数、原始扫描时长以及扫描时间窗的 IMU 覆盖。录包第一帧缺少更早的 IMU 只记作 `start_unknown`。
- 回放使用 `ros2 bag play --clock 100 -r 1.0`。脚本先启动 FAST-LIO2、监测节点和里程计订阅者，等待 5 秒再播放；bag 结束后再等 4 秒收尾。FAST-LIO2 使用 `use_sim_time:=true`、`rviz:=false`，并关闭地图文件保存。
- `odometry_summary.json` 的 `closure_3d_m` 是 FAST-LIO2 第一条和最后一条 `/Odometry` 位姿之间的欧氏距离；`trajectory_length_m` 是相邻估计位姿的累计距离。它们衡量本次估计轨迹的闭合程度，不是对真实轨迹精度的测量。
- 回放启动或结束时监测节点的 `NO_DATA`、`START_UNKNOWN` 与临时接收间隔可能来自录包边界或播放调度；判断原始录制是否缺帧时，以 `*_inputs.json` 的传感器时间戳检查为准。

逐项证据位于每组结果目录：`odometry.csv`、`odometry_summary.json`、`monitor.log`、`fast_lio.log`、`player.log` 和 `run_status.json`。三组轨迹图为 `trajectories.png`。不同机器的线程调度可能使里程计末值略有差异，复测时应比较量级、消息覆盖和异常位置。

## 时间戳对齐审计

`audit_bag_timing.py` 直接读取 MCAP，核对点云 `header.stamp`、`timebase`、逐点偏移、IMU 时间戳，以及每帧扫描起终点附近的 IMU 样本。它也统计 MCAP 录制时间与消息时间戳的差异。审计不会改 FAST-LIO2 参数或原始录包。以室内静止包为例：

```bash
cd ~/lidar_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 evaluation/audit_bag_timing.py \
  bags/室内静止 evaluation/results/indoor_static_timing.json
```

另外两包分别将路径和输出名换为 `bags/室内闭环` / `indoor_loop_timing.json` 与 `bags/室外闭环` / `outdoor_loop_timing.json`。三组现有录包的详细结果和判断边界见 `evaluation/TIMING_AUDIT_2026-10-06.md`。相邻 IMU 样本距离扫描边界只有几毫秒，**不等于**已测得 LiDAR–IMU 物理偏移。录包没有保存原始包的时间同步类型，不能单凭这些 JSON 把 `time_offset_lidar_to_imu` 判为零。
