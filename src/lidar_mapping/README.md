# lidar_mapping

独立的单会话占用地图适配包，使用本机 OctoMap 1.9.7 的概率占用和射线遍历。ROS 消息、按扫描时间的 TF 与输出由 `mapping_node` 管理；纯 C++ `OccupancyMapper` 不依赖 ROS。

- 输入：相同扫描时间的 `/perception/ground`、`/perception/obstacles`、`/perception/clearing`
- 坐标：`odom`；射线起点是 `nav_sensor` 的真实 LiDAR 原点
- 输出：持久化 `/map`，三态二维图为未知 -1、真实重复地面支持的自由 0、占用列 100
- 服务：`/mapping/save_map`，保存 PGM/YAML 和非破坏性 `.bt` 副本；正常退出也保存
- 默认配置：`config/mapping.yaml`；局部地图独立运行，不等待全局积分

未观测空间不会因经过一条三维射线就被投影为可通行地面。占用列优先于自由列，真实自由射线可以降低旧占用概率；没有新观测不会自动删除旧障碍。动态物体未做语义剔除，没有回环优化、重定位或坡地/负障碍通行保证。

对外入口：`ros2 launch lidar_nav2_bringup lidar.launch.py mode:=mapping`。需自行回放录包；运行环境通过工作区的 `scripts/setup_lidar_nav.sh` 加载。

保存的 `odom` 地图不能直接用于另一次真实启动。当前加载验证使用相同录包重复初始化，不能证明跨会话定位能力。接入硬件前需要真实安装几何、机器人足迹与定位后端。

OctoMap 开源来源：https://github.com/OctoMap/octomap 。默认 YAML 已启用原生逐路径占用更新和按变更键维护二维列缓存；完整投影会展开粗叶节点覆盖范围。真实端点预算保持 0（全部保留），积分 2 Hz、发布 1 Hz、分辨率 0.10 m 和地面证据阈值保持原值。单独设 `incremental_projection: false`、`incremental_inner_updates: false` 可恢复完整扫描实现；`prune_interval: 0` 关闭独立全树裁剪。

配对订阅缓存为 20 帧，待配对容量为 64 帧，超时仍为 1 s。诊断输出阶段耗时、各话题接收计数、缺失话题及未配对扫描 ID；地面门控造成的缺失不能当作通信丢包。

445 帧相同真实建图输入上，默认实现的二维地图逐帧与完整扫描基线一致。真实 1× 回放中建图 CPU 时间约减半；参数实验的射线抽样和积分减半均削弱原有效路径，未设为默认。完整性能、质量与规划边界见 `evaluation/map_quality/PERFORMANCE_2026-10-09.md`。
