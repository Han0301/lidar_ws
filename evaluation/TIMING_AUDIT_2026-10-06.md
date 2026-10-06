# MID-360 三组录包的时间戳核查（2026-10-06）

## 判断

三组 MCAP 的 LiDAR 与 IMU 消息时间戳均严格递增；点云 `header.stamp` 与 `timebase` 完全一致，所有帧的最早点偏移量为 0。除每包首帧缺少包内更早的 IMU 外，扫描起终点均有 IMU 样本夹住，IMU 流没有超过 15 ms 的间隔。这证明录包内消息的时间结构可供 FAST-LIO2 消费，**不能证明 LiDAR 与 IMU 的真实物理时间偏移为 0 ms**。

当前 FAST-LIO2 配置是 `time_sync_en: false`、`time_offset_lidar_to_imu: 0.0`。这是运行假设，现有 ROS 消息尚未验证该假设。因此本轮没有改同步参数，也没有据闭合路径首末距离挑选新的精度参数。

## 逐包数据

下表的“起点前”“终点后”是扫描边界到相邻 IMU 样本的**采样间隔中位数**，不能解释为两传感器的时间偏移。

| 录包 | LiDAR / IMU 条数 | 起点前 IMU | 终点后 IMU | 最大 IMU 间隔 | 超过 150 ms 的 LiDAR 间隔 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 室内静止 | 730 / 14,590 | 3.06 ms | 2.45 ms | 8.16 ms | 0 |
| 室内闭环 | 941 / 18,814 | 1.34 ms | 2.71 ms | 8.71 ms | 0 |
| 室外闭环 | 2,594 / 52,158 | 1.16 ms | 4.41 ms | 6.83 ms | 1 次，1,400.00 ms |

三包的 `point_num` 与实际点数也全部一致。每包只有第一帧缺少更早的包内 IMU 样本；所有帧都有扫描终点后的 IMU 样本。室外包的 LiDAR 间隔发生在第 16 帧后、距首帧 1.50 s 处，同期 IMU 继续递增。

录包时间减消息时间戳的中位数，三包 LiDAR 分别为 103.02、102.67、102.60 ms，IMU 分别为 0.084、0.116、0.112 ms。LiDAR 消息要等约 100 ms 扫描积累后才发布，所以这组差值也不是 LiDAR–IMU 的物理偏移。室外包前 16 帧的 LiDAR 录制延迟从约 3.005 s 逐帧降到约 1.547 s，随后出现 1.400 s 点云间隔；从消息与 MCAP 不能定位积压发生在设备、驱动还是录包侧。

## 驱动时间来源与证据缺口

固定版本 Livox 驱动的 `src/comm/pub_handler.cpp` 对点云包和 IMU 包都调用 `GetEthPacketTimestamp()`。当原始包 `time_type` 表示 PTP/gPTP 或 GPS 时，使用包内纳秒时间戳；否则回退到主机 `high_resolution_clock` 的当前时间。`src/lddc.cpp` 把点云首点时间写入 `CustomMsg.timebase` 和 `header.stamp`，点内使用相对时间；IMU 的 `header.stamp` 来自同一处理路径的包时间戳。[Livox 官方协议](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md)定义了原始包的 `time_type`，但当前 ROS 消息与 MCAP 没有保存它。因此无法从现有三包确认当时使用的是设备同步时间还是主机接收时间，也无法独立测出固定时间偏移。

## 下一步的测量门槛

（1）实时采集时同步保存原始 UDP 包的 `time_type` 或驱动时间来源诊断，并记录主机与设备的 PTP/GPS 状态；先确认是否全程使用同一时间基准。

（2）另录有明显三轴旋转激励的标定数据，用独立的 LiDAR–IMU 时间/外参标定方法求偏移；至少重复两次，并检查结果在独立录包上是否稳定。官方 [LI-Init](https://github.com/hku-mars/LiDAR_IMU_Init) 支持 MID-360 的时间与外参初始化，但其代码环境要与当前 ROS 2 Jazzy 工作空间隔离验证。

（3）只有来源和偏移核查通过后，才在同一批有物理标记或真值的数据上做参数 A/B，并同时比较轨迹/地图质量及逐帧处理延迟。现有三包可以继续用于处理速度基线，但不能单凭首末估计位置选“更准”的参数。

详细数值由 `evaluation/audit_bag_timing.py` 从原始 MCAP 重新读取；该脚本不修改 FAST-LIO2、驱动或原始录包。
