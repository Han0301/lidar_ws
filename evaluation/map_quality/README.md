# 建图、局部地图质量与性能评测

本目录只包含独立评测工具和实验配置。复用 FAST-LIO2、感知、占用建图及 Nav2 节点，采集地图质量、阶段耗时、CPU/RSS 和固定输入输出；不发送底盘运动命令。生产实现位于 `src/`，评测输出、实测报告和现场图片保存在本地。

## 1 文件与职责

| 文件 | 用途 |
| --- | --- |
| `quality.yaml` | 评测参数、已有证据路径、随机种子、ROI 与一致性阈值 |
| `run_quality.py` | 隔离通信域回放、复制实际安装配置、记录哈希、备份恢复 FAST-LIO 日志 |
| `record_quality.py` | 只读采集原始/处理点云时间、诊断、TF、几何框、成功更新 footprint、完整原始代价地图 |
| `common.py` | 统计、PCD 读取、RANSAC/TLS、世界坐标滚动网格对齐 |
| `analyze_quality.py` | 感知门控、结构异常、地图更新/发布、栅格转换、观测年龄、观察器回调差值 |
| `analyze_geometry.py` | 保存 ROI、全区域/内点残差、厚度、固定平面逐帧投影、重复观测与结构障碍 ROI |
| `summarize_prior.py` | 从旧 CSV/JSON 复算轨迹、规划和官方有效性，不重跑旧实验 |
| `score_annotations.py` | 仅在提供真实人工/测量栅格标签后计算 Precision、Recall、IoU |
| `test_metrics.py` | 五个指标实现正确性测试，合成输入不计入机器人实测 |
| 本地 `RESULTS_*.md`、`OPTIMIZATION_*.md`、`PERFORMANCE_*.md` | 各轮实测报告、指标定义及局限，不随代码提交 |

## 2 直接复算已保存结果

环境已有 NumPy、SciPy、PyYAML、Matplotlib、ROS 2 Jazzy 和本地 Nav2 1.3.13。不需要编译算法。

```bash
cd /home/h/lidar_ws
source scripts/setup_lidar_nav.sh
export MPLCONFIGDIR=/tmp/lidar_quality_mpl
export OPENBLAS_NUM_THREADS=1
Q=evaluation/map_quality
/usr/bin/python3 "$Q/test_metrics.py"
/usr/bin/python3 "$Q/summarize_prior.py" --output "$Q/results/analysis/prior"
/usr/bin/python3 "$Q/analyze_quality.py" \
  --run "$Q/results/outdoor_20261008_v2" --output "$Q/results/analysis/outdoor"
/usr/bin/python3 "$Q/analyze_geometry.py" \
  --capture "$Q/results/outdoor_20261008_v2/capture" \
  --map "$Q/results/outdoor_20261008_v2/capture/published_map.npz" \
  --output "$Q/results/analysis/outdoor_geometry"
/usr/bin/python3 "$Q/analyze_geometry.py" \
  --capture "$Q/results/indoor_20261008/capture" \
  --map "$Q/results/indoor_20261008/capture/published_map.npz" \
  --output "$Q/results/analysis/indoor_geometry"
/usr/bin/python3 "$Q/analyze_geometry.py" \
  --map maps/fastlio_mid360.pcd --output "$Q/results/analysis/legacy_pcd"
```

旧轨迹的实际保存位置位于上一轮 Codex 工作目录，见 `quality.yaml`。在其他机器上应把原始证据复制到可用位置并调整该路径；不要用当前轨迹覆盖旧轮次。几何 ROI 自动发现是固定种子、固定规则的区域选择；后续 A/B 必须用基线保存的 `rois.json`，通过 `--rois` 固定 ROI，禁止各自选择更好的区域。

## 3 新采集

先停止正在运行的 FAST-LIO2。脚本拒绝已有 FAST-LIO 进程，结果目录必须不存在；两个回放按顺序运行。默认域 231、1× 回放，不接实时驱动。

```bash
cd /home/h/lidar_ws
source scripts/setup_lidar_nav.sh
Q=evaluation/map_quality
/usr/bin/python3 "$Q/run_quality.py" --bag bags/室外闭环 --navigation \
  --output "$Q/results/outdoor_repeat_01"
/usr/bin/python3 "$Q/run_quality.py" --bag bags/室内闭环 \
  --output "$Q/results/indoor_repeat_01"
```

运行输出复制安装目录的感知/桥接/地图配置。FAST-LIO 的数学处理参数沿用源码 YAML，与已有离线测试一样关闭 PCD 磁盘保存，仅重定向输出路径；`runtime_pos_log_enable` 未开启。Nav2 仅启用原生日志计时，不修改更新/发布频率。原始日志备份、该次日志及哈希均保存。生产 `src/FAST_LIO/Log` 的已有文件在收尾恢复。

`run_status.json` 分别记录采集是否完整和收尾异常；新脚本对收尾异常返回非零。2026-10-08 室外数据在主动 SIGINT 后出现独立 costmap 清理段错误，详见报告。重新采集若发生此问题，保留日志，不得改写为稳定性通过。异常退出发生在播放期间时，分析器拒绝以完整数据分析。

`record_quality.py` 使用后台有界队列保存数据。原始点云和各处理话题回调的到达时间来自同一个观察器执行器，可能因排队出现负差值；CSV 保留这些样本。它们不能替代生产回调或 DDS 事件的独立追踪。栅格消息时间为发布时钟，无法确认每格用了哪一扫描。

## 4 补充真值与可执行评分

先在选定真实静态帧的世界坐标栅格上人工标出障碍和自由区域，或使用经测量、已配准的障碍位置。保存 NPZ：`occupied` 为 bool 障碍标签；`evaluation_mask` 为 bool 已标注区域；`origin`、`resolution` 与原始预测地图一致；`provenance` 为非空字符串，写明标注人、时间、测量/标注方法和帧号。未标注区域不可放入 mask。动态对象必须使用同一帧标签；膨胀格不能自动当障碍真值。

```bash
Q=/home/h/lidar_ws/evaluation/map_quality
/usr/bin/python3 "$Q/score_annotations.py" \
  --prediction "$Q/results/outdoor_20261008_v2/capture/costmap_1791211035184255135.npz" \
  --truth /absolute/path/to/reviewed_cells.npz --output "$Q/results/reviewed_occupancy_scores.json"
```

预测文件名应替换为实际被标注帧，报告内没有任何现成真值或伪造标注。该评分只针对标注区域的几何障碍占用，不验证底盘可通行性。要验证同一静态障碍物身份和范围一致性，应先人工确认 `rois.json` 中的区域、可见时段和对象身份，再使用 `--rois` 重算；本次输出仍明确标为结构候选。

真正的传感器至地图端到端时延需要独立记录 DDS 接收/生产回调开始结束，并在地图层记录所消费的扫描时间戳；可先在测试构建中做追踪，不改变算法数学逻辑。本次该指标为空，不能由“最新观测年龄”代替。实时设备应另建目录、设 `use_sim_time=false`、记录实时驱动/CPU负载/时钟来源，不能与本次录包结果合并。

## 5 性能与固定输入回归

当前性能配置使用独立参考采样、10 Hz 局部体素地图及单会话 OctoMap 全局图。`run_quality.py` 根据所选配置运行实验；旧默认配置用于原始质量采集，完整性能配置显式选择如下：

```bash
cd /home/h/lidar_ws
source scripts/setup_lidar_nav.sh
python3 evaluation/map_quality/run_quality.py \
  --config evaluation/map_quality/performance_optimized_final.yaml \
  --bag bags/室外闭环 --navigation \
  --output evaluation/map_quality/results/performance_repeat_01
```

结果目录必须不存在，完整回放串行运行。YAML 中的工作空间、旧证据及组件路径属于当前主机配置，在其他机器上需对应调整；原始 bag、历史证据和旧实现归档需另行提供。

| 工具 | 用途 |
| --- | --- |
| `resource_profile.py`、`analyze_performance.py` | 进程 CPU/RSS 采样及阶段耗时汇总 |
| `prepare_performance.py`、`benchmark_mapping.cpp` | 保存同一扫描输入与变换，执行纯 C++ 建图基准 |
| `build_performance_tools.sh`、`compare_perception_performance.py` | 用归档旧实现与当前实现处理同一批真实强度点云 |
| `run_mapping_sweep.py`、`mapping_sweep.yaml` | 对比全树/增量投影、裁剪与积分预算候选 |
| `compare_mapping_stability.py`、`compare_performance_geometry.py` | 同时刻栅格稳定性与障碍几何比较 |
| `test_costmap_shutdown.py`、`test_system_interface.py` | 生命周期退出、地图加载、规划及官方路径有效性复核 |
| `plot_performance.py`、`write_performance_report.py` | 从本地结果生成图片及 Markdown 报告 |

固定输入工具构建需要额外提供旧源码归档目录，不能把重新运行当前生产代码当作旧实现基线。阶段墙钟耗时、进程 CPU 时间和传感器至地图的端到端时延分别解释；输出一致性与官方路径有效性不等同真实场地精度或底盘安全验证。
