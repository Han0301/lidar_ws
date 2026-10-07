# MID-360 感知与 Nav2 接入

## 当前功能

在原有 FAST-LIO2 之后增加独立的感知支路：去畸变点云 → 重力对齐 → 可选操作者区域过滤 → 体素降采样 → 地面分割（ground segmentation）→ 几何聚类 → Nav2 代价地图（costmap）。原始 Livox 数据仍直接进入 FAST-LIO2。

户外录包用于验证点云、坐标、地图和路径规划；导航控制闭环用 Nav2 官方 loopback 仿真验证。手持录包没有底盘反馈，不能用于证明实车导航成功。

## 构建与快速开始

首次部署先完成根目录 README 的上游依赖与 Livox SDK 构建。已有工作空间按以下步骤构建新包：

```bash
cd ~/lidar_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/lidar_perception src/lidar_nav2_bringup --ignore-src -r -y
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --merge-install --executor sequential \
  --packages-select lidar_perception lidar_nav2_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source scripts/setup_lidar_nav.sh
```

当前电脑的 Nav2 运行依赖位于已忽略的 `.deps/nav2`，环境脚本会自动加入路径；其他电脑可通过上面的 `rosdep` 安装。依赖不是源码仓库内容。

启动原有 FAST-LIO2 后，再启动感知支路：

```bash
source ~/lidar_ws/scripts/setup_lidar_nav.sh
ros2 launch lidar_nav2_bringup perception.launch.py
```

默认 YAML 使用录包仿真时间（simulation time）；连接在线雷达时，把 `bridge.yaml` 和 `perception.yaml` 中的 `use_sim_time` 都改为 `false`，并确认静止初始化、话题和外参。工作空间没有新增底盘节点。

完整点云到导航闭环的仿真入口：

```bash
cd ~/lidar_ws
source scripts/setup_lidar_nav.sh
ROS_DOMAIN_ID=220 python3 evaluation/lidar_navigation/run_simulation.py
```

该入口自动启动 Nav2、理想点云生成、感知节点、起点和目标，并保存路径与结果；点云生成器只属于评估工具。

## 代码与话题

| 位置 | 责任 | 核心输入 | 核心输出 |
| --- | --- | --- | --- |
| `src/lidar_perception/src/cloud_pipeline.cpp` | 有效点、区域过滤、体素、地面、聚类 | body 点云、对应时刻旋转、区域 | 地面、障碍、真实清除终点、几何框 |
| `src/lidar_perception/src/ground_segmenter.cpp` | 低处点群拟合重力约束平面，再约束 Patchwork++ 局部地面 | 去畸变并调平的点 | 地面参考及可信状态 |
| `src/lidar_perception/src/perception_node.cpp` | 有界 TF 等待、消息转换、诊断、地图输入门控 | `/cloud_registered_body`、`/perception/operator_regions` | `/perception/ground`、`obstacles`、`clearing`、`operator_removed`、`objects`、`markers`、`diagnostics` |
| `src/lidar_nav2_bringup/src/lio_nav_bridge.cpp` | 初始化重力方向、保留 LIO 位姿并投影平面参考 | `/livox/imu`、`/Odometry`、`/perception/ground_reference` | `/nav/odom`、TF、`/nav/diagnostics` |
| `src/lidar_nav2_bringup/config` | 感知到 Nav2 的坐标、体素、地图、规划与控制参数 | YAML | 标准 Nav2 节点 |
| `evaluation/lidar_navigation` | 录包、候选区域、离线参数对照、理想仿真、结果采集 | 已录数据或仿真 | 证据文件 |

```text
/livox/lidar + /livox/imu ── FAST-LIO2 ── /cloud_registered_body
                                              │
                                 TF 对齐 + 可选区域过滤
                                              │
                                 地面分割 + 几何聚类
                                              │
                     可信障碍标记 + 原始观测终点清除
                                              │
                         Nav2 VoxelLayer + InflationLayer
                                              │
                              NavFn → DWB → 仿真运动反馈
```

坐标变换（transform，TF）树：`map → odom → camera_init → body`；`odom` 另发布 `nav_base`、`perception_sensor`、`nav_sensor`。`camera_init` 到 `body` 继续由 FAST-LIO2 发布。`nav_base` 是手持数据的虚拟平面参考；`map → odom` 固定为单位变换，尚未实现重定位或全局漂移修正。

点云和位姿均使用扫描结束时间戳。启动前先积累 100 个低角速度 IMU 样本；瞬时运动加速度不用于逐帧重新确定重力。首次可信地面观测直接建立导航高度参考，不从零高度施加跳变限制；后续高度跳变超出 YAML 阈值时保留旧参考并报告 `GROUND_REFERENCE_JUMP`。它提示重新核查地形与观测，不能当成可通行判断。物理 LiDAR–IMU 时钟偏移仍未被独立证明，保持原 LIO 配置。

## 过滤操作者的边界

生产默认不启用身份未经确认的删除区域。`operator_candidate.yaml` 保存从户外近场点云提出的持有者区域候选，只供人工检查和对照。

`OperatorFilter::excludes()` 仅删除指定空间盒内的点，不识别人、不跟踪身份。第二位同学需要明确区域或经过检查的轨迹标签。`operator_annotations.py` 根据人工标签发布同一扫描时刻的区域；过期或坐标不可用的区域不参与删除。不能据此宣称全部人体已经正确移除。

删除只影响感知支路；它没有清理 FAST-LIO2 已生成的地图，也没有证明定位更准。清除点云保留包括操作者在内的原始测量终点，不对缺失点向远处补射线。

## 导航地图的规则

- 默认感知体素 0.10 m，点距离 0.45–15 m；障碍高度相对于可信地面为 0.12–2.50 m。低于 12 cm 的物体不在本次检测能力声明内。
- Patchwork++ 局部地面点还必须满足参考平面的 0.16 m 偏差约束；这是平坦或缓变地面配置，台阶、沟坑和大坡度通行性尚未验证；本包在约 0.6 m 台阶上启动只是现场信息，不是任何算法参数。Patchwork++ 内部 1.3 m 是输入归一化的参考值，实际传感器离地高度由点云估计。
- 地面参考不足时保留原始观测用于诊断，同时暂停 Nav2 标记和清除点云。标准观察缓冲区约 0.5 s 后会过期，规划不能把这种状态视为可靠自由空间。
- 标记只使用过滤后的非地面点；清除使用真实观测终点，射线从实际传感器原点开始。没有观测的区域保持未知，规划器 `allow_unknown: false`。
- Nav2 的高度边界属于全局坐标。当前体素纵向覆盖约 −2.0–3.6 m；不能把手持雷达的地面直接假定为全局 Z=0。
- 局部地图 12×12 m、分辨率 0.10 m；虚拟圆形 footprint 半径 0.25 m，膨胀半径 0.55 m。它们是评估假设，实际机器人必须使用自己的几何与运动限制。
- 录包局部地图通过 `bag_costmap.launch.py` 启动；其生命周期由 `activate_costmap.py` 在时钟和位姿可用后启用。该独立 costmap 无父服务器生命周期 bond，管理器关闭 bond 等待；规划服务器保留标准 bond。
- `bag_planner.launch.py` 使用同一观测建立滚动规划地图，只规划已经观测的邻近空间，不输出速度。

## 官方 IsPathValid 快照评估

`test_official_ispathvalid.py` 读取同源对照保存的 `paired_planning.json` 和原始代价地图，调用安装版本的 `/official_snapshot/is_path_valid`。`ispathvalid_fixture` 继承官方 PlannerServer，只加载快照、设置该次起点的 TF 并暂停地图更新；没有重写 `isPathValid`。每张地图在调用前后通过官方 GetCostmap 逐格核对，同时检查原点和分辨率。

该工具限定圆形轮廓配置（半径 0.25 m、padding 0.01 m），用保存的 XY 路径和单位姿态。插件更新已暂停，没有回放运动、重新规划或速度输出。空路径、地图外位置以及自由/内切膨胀/致命障碍/未知格采用独立单点请求作为对照；对被拒绝路径逐点调用官方服务定位，单点索引与完整路径响应的 `invalid_pose_indices` 分开记录。

```bash
cd ~/lidar_ws
source scripts/setup_lidar_nav.sh
cmake -S evaluation/lidar_navigation/ispathvalid_fixture \
  -B evaluation/lidar_navigation/results/tools/ispathvalid_fixture \
  -DCMAKE_BUILD_TYPE=Release
cmake --build evaluation/lidar_navigation/results/tools/ispathvalid_fixture -j2
REFERENCE_DIR="/absolute/path/to/saved/planning"
ROS_DOMAIN_ID=230 python3 evaluation/lidar_navigation/test_official_ispathvalid.py \
  --reference "$REFERENCE_DIR" \
  --fixture evaluation/lidar_navigation/results/tools/ispathvalid_fixture/ispathvalid_fixture \
  --output evaluation/lidar_navigation/results/official_ispathvalid
```

Nav2 不同版本的服务与判断条件可能不同，应保留本机接口、库版本和控制请求响应。`allow_unknown: false` 是 NavFn 参数；不要假定它改变 IsPathValid 服务的未知格策略。检查通过也不覆盖真实地形通行、路径点之间的连续扫掠或实际底盘碰撞。参考 [Nav2 1.3.13 PlannerServer 源码](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_planner/src/planner_server.cpp)。

## 复用的开源方案

- [PCL 聚类](https://pointclouds.org/documentation/tutorials/cluster_extraction.html)：体素、平面拟合与欧氏聚类。
- [Patchwork++](https://github.com/url-kaist/patchwork-plusplus)：地面分割核心，固定提交和 BSD 许可证见 vendor/patchworkpp/UPSTREAM.md。
- [Autoware 地面分割](https://autowarefoundation.github.io/autoware_universe/latest/perception/autoware_ground_segmentation/docs/scan-ground-filter/)：参考预处理、地面与非地面职责划分；其固定车体假设不直接套用到手持数据。
- [Nav2 地形与地面一致性方案](https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_ground_consistency_layer/navigation2_with_ground_consistency_layer/)：参考局部地面高度与地面/非地面证据融合，作为复杂地形的后续候选；候选配置通过固定版本的外部插件与本地补丁集成；默认仍使用原生体素地图。插件回归与户外对照分别验证，不能将插件自身的地面证据规则当作地形通行真值。
- [Nav2 VoxelLayer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/voxel/)：使用标准标记、射线清除与膨胀接口。

运行后可在本机结果目录检查实际输出。场地数据、图片和实测报告默认不上传远程仓库。

录包评估入口如下；产物保存在已忽略的 results 下，不进入 Git：

```bash
cd ~/lidar_ws
source scripts/setup_lidar_nav.sh
python3 evaluation/lidar_navigation/run_navigation_bag.py bags/室外闭环 outdoor 223
```

规划测试保存各次起终点、时间、地图快照与结果；成功回放不等于全部规划成功。本次未部署实车、未验证真实人体识别精度，也没有独立轨迹或地图真值。


## 地面证据候选地图

候选支路用于检查原生三维射线无法及时清除的障碍残留。按 [Nav2 官方教程](https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_ground_consistency_layer/navigation2_with_ground_consistency_layer/)接入 [DFKI 地面一致性插件](https://github.com/dfki-ric/nav2_ground_consistency_costmap_plugin)，固定提交 `41cec620efba6c370dccfc59a6ec1134775ff48a`，上游 BSD 许可证保留在依赖目录。本地补丁为 `patches/ground_consistency.patch`。

```text
同一可信感知流
  ├─ /perception/clearing → ray_layer（只清除真实终点）
  └─ /perception/ground + /perception/obstacles → ground_consistency
                                                          ↓
                                                     inflation_layer
                                                          ↓
                                                       NavFn
```

自由空间需要地面证据支持；证据衰减本身产生未知，新出现的非地面点立即参与障碍判断。旧地面高度仅在其证据仍足够时参与局部/邻域判断；输入停止时不衰减。补丁同时保留前置地图层的致命占用。相对于上游默认值，候选 YAML 使用有上限的证据积累和更短的障碍记忆；这些参数需要结合实际传感器密度与地图更新频率评估。

先构建候选依赖，再加载环境：

```bash
cd ~/lidar_ws
bash scripts/build_ground_consistency.sh
source scripts/setup_lidar_nav.sh
```

在已运行的感知回放旁启动候选规划器：

```bash
ros2 launch lidar_nav2_bringup bag_planner.launch.py params:="$PWD/src/lidar_nav2_bringup/config/bag_planner_ground.yaml"
```

默认启动仍使用 `bag_planner.yaml`。候选配置的高度、圆形轮廓、膨胀参数是评估假设，必须按实际底盘重新设置；复杂地形、负障碍、定位漂移和操作者身份过滤尚未通过完整验证。

## 同源对照与地图回归

`run_paired_audit.py` 等待回放时钟、导航位姿和 `odom→nav_base` 可用后启动两个名字空间的规划器。`dual_planner_test.py` 检查代价地图生命周期已激活，并确认滚动地图覆盖当前位姿；每组暂停回放后使用相同显式起终点、保存两份原始代价地图和路径。两份地图共用传感器流，但每次服务快照并非规划器内部地图的逐字复制。

```bash
LIDAR_EVAL_PAIRED=1 python3 evaluation/lidar_navigation/run_navigation_bag.py bags/室外闭环 outdoor_paired 223
```

ROS 仿真时钟暂停时，动作结果中的 `planning_time` 可能为零，不能用作真实计算耗时。评估另存单调墙钟的动作请求至结果回调延迟；它包括通信、执行器调度与客户端请求次序影响，不是纯规划算法耗时；不能据此证明算法速度提升。

`layer_regression` 使用实际 Nav2 插件构造六类确定输入：持续低障碍、离开后地面重新出现、原位置没有观测、输入停止、弱地面证据、已累积地面上新出现低障碍。候选插件断言失败将返回非零；它是组件回归，不是人体识别或真实通行性评估。

```bash
cmake -S evaluation/lidar_navigation/layer_regression -B evaluation/lidar_navigation/results/tools/layer_regression -DCMAKE_BUILD_TYPE=Release
cmake --build evaluation/lidar_navigation/results/tools/layer_regression -j2
evaluation/lidar_navigation/results/tools/layer_regression/layer_regression evaluation/lidar_navigation/results/layer_guarded.csv
```

## 演示视频来源

`export_demo.py` 将保存的点云、原始地图、规划路径和仿真里程计重绘为 MP4。视频明确标注录包快照、抽样时间和理想仿真边界，不将手持路线动画当作机器人执行结果。需要 Python NumPy、Pillow、带 H.264 MP4 编码支持的 OpenCV，以及 Noto Sans CJK 字体；字体路径位于脚本 `FONT` 常量。

```bash
python3 evaluation/lidar_navigation/export_demo.py --paired evaluation/lidar_navigation/results/outdoor_paired --perception evaluation/lidar_navigation/results/outdoor_paired --simulation evaluation/lidar_navigation/results/simulation_acceptance --output evaluation/lidar_navigation/results/demo
```

输出包含地图对照视频、地面与障碍点云视频、理想仿真轨迹视频，以及解码校验元数据。场地视频和实测数据保持本地保存，公开发布需单独授权。
