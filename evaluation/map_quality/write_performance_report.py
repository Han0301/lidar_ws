"""Build a source-backed Markdown/CSV summary from completed experiments."""
import argparse
import csv
import json
import shutil
from pathlib import Path
import yaml
from common import sha256,write_json,write_csv

p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args();root=a.root.resolve()
perf=json.loads((root/'performance.json').read_text());comp=json.loads((root/'comparison/comparison.json').read_text());paired=json.loads((root/'perception_paired.json').read_text());core=perf['mapping_core']
shutdown=json.loads((root/'shutdown_direct/results.json').read_text())
assert all(r['exit_code']==0 and not any(r['transitions']) for r in shutdown['cases'])
interfaces={n:json.loads((root/('interface_'+n+('_direct' if n in ['ray','half'] else ''))/'results.json').read_text()) for n in ['full','cache','prune','native','ray','half','final']}
assert all(r['measurement_complete'] and all(r[k]==0 for k in ['player_returncode','system_returncode','lio_returncode']) and all(v==0 for k,v in r.items() if k.startswith('/lifecycle_')) for r in interfaces.values())
profiles=['full','cache','prune_cache','native_cache','ray_budget_2000','half_integration']
labels=dict(full='完整扫描',cache='仅二维缓存',prune_cache='缓存＋每20帧裁剪',native_cache='原生路径更新＋缓存',ray_budget_2000='真实终点预算2000',half_integration='积分帧减半')
def table(headers, rows):
 return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+['| '+' | '.join(map(str,r))+' |' for r in rows])
def phase(n,k):return perf[n]['timings'][k]
def pct(b,c):return 100*(b-c)/b
online=[]
for n in ['baseline','optimized','optimized_final']:
 r=perf[n];m=comp[n]['run_metrics'];g=comp[n]['global_map'];s=comp[n]['matched_cadence_stability']['slow_sensor']
 online.append(dict(profile=n,processed=m['perception']['processed'],gate_valid_percent=100*m['perception']['gate_valid_fraction'],
     mapping_cpu_s=r['resources']['cpu_seconds']['mapping'],sampled_peak_rss_mib=r['resources']['sampled_peak_rss_mib']['mapping'],
     integration_p95_ms=phase(n,'lidar_mapping/integration/integration_ms')['p95'],projection_publish_p95_ms=phase(n,'lidar_mapping/projection_publish_ms')['p95'],
     slow_retention_percent=100*s['lethal_retention_fraction']['mean'],slow_toggle_percent=100*s['lethal_toggle_fraction']['mean'],
     slow_aba_mean=s['aba_lethal_flicker_cells']['mean'],free_cells=g['free_cells'],occupied_cells=g['occupied_cells'],unknown_percent=100*g['unknown_fraction']))
write_csv(root/'online_summary.csv',online)
write_csv(root/'core_summary.csv',[dict(profile=n,integrated=core[n]['integrated'],exact_grid_frames=core[n]['exact_grid_frames'],
    cpu_s=core[n]['resources']['cpu_s'],peak_rss_mib=core[n]['resources']['peak_rss_mib'],integration_p95_ms=core[n]['performance']['integration_ms']['p95'],
    project_p95_ms=core[n]['performance']['project_ms']['p95'],**core[n]['final_geometry']) for n in profiles])
for n in ['baseline','optimized','optimized_final']:
 assert json.loads((root/n/'run_status.json').read_text())['measurement_complete']
planning=[]
for n,j in interfaces.items():
 for group in ['planning','fixed_baseline_planning']:
  for i,r in enumerate(j.get(group,[])):
   planning.append(dict(profile=n,group=group,index=i,source=r['source'],error_code=r['error_code'],official_valid=r.get('official_valid'),path_length_m=r.get('path_length_m'),start=json.dumps(r['start']),goal=json.dumps(r['goal'])))
write_csv(root/'official_planning.csv',planning)
write_json(root/'suite_status.json',dict(full_replays={n:json.loads((root/n/'run_status.json').read_text()) for n in ['baseline','optimized','optimized_final']},
    core_profiles=profiles,core_tests=dict(perception=7,mapping=8,failures=0),metric_tests=5,shutdown_direct=shutdown,
    interface_tests={n:dict(complete=j['measurement_complete'],process_exit_codes={k:j[k] for k in ['player_returncode','system_returncode','lio_returncode']},shutdown_service_codes={k:v for k,v in j.items() if k.startswith('/lifecycle_')}) for n,j in interfaces.items()},
    selected_mapping_config=yaml.safe_load(Path('src/lidar_mapping/config/mapping.yaml').read_text()),
    rejected_defaults=['periodic_prune','ray_budget_2000','half_integration'],
    retained_failure_evidence=['optimized (5 missing ground frames)','shutdown_regression (blocked CLI daemon, transitions not executed)','interface_ray/half (fresh participant shutdown service failure; retried in *_direct)']))
write_json(root/'core_manifest.json',dict(settings=yaml.safe_load(Path(__file__).with_name('mapping_sweep.yaml').read_text()),
    binary_sha256={n:sha256(root/n) for n in ['benchmark_mapping','benchmark_baseline','benchmark_optimized']},
    compiled_mapping_benchmark_source_sha256=sha256(root/'tools_source/benchmark_mapping.cpp'),
    input_manifest_sha256=sha256(root/'input/manifest.json'),method='serial replay, source stamps fixed, geometry and probability thresholds unchanged'))
checks=root/'checks';checks.mkdir(exist_ok=True)
for source,name in [('/tmp/lidar_perf_release_build.log','build.log'),('/tmp/lidar_perf_release_tests.log','core_tests.log'),('/tmp/lidar_perf_metrics_test.log','metric_tests.log'),('/tmp/lidar_perf_shutdown_direct.log','shutdown_direct.log'),('/tmp/lidar_perf_perception_test_result.log','perception_test_result.log'),('/tmp/lidar_perf_mapping_test_result.log','mapping_test_result.log')]:shutil.copy2(source,checks/name)
b,f=online[0],online[2];truth=json.loads((root/'ros_geometry.json').read_text());stable=json.loads((root/'mapping_stability.json').read_text())
core_rows=[[labels[n],core[n]['integrated'],f"{core[n]['resources']['cpu_s']:.2f}",f"{core[n]['resources']['peak_rss_mib']:.1f}",f"{core[n]['performance']['integration_ms']['p95']:.2f}",f"{core[n]['performance']['project_ms']['p95']:.2f}",str(core[n]['exact_grid_frames'])+'/445'] for n in profiles]
geo_rows=[[labels[n],core[n]['final_geometry']['free_cells'],core[n]['final_geometry']['occupied_cells'],f"{100*core[n]['final_geometry']['unknown_fraction']:.2f}%",core[n]['final_geometry']['largest_free_component_cells'],core[n]['final_vs_full']['different_cells']] for n in profiles]
local_rows=[[n,f"{r['gate_valid_percent']:.2f}%",f"{r['slow_retention_percent']:.2f}%",f"{r['slow_toggle_percent']:.3f}%",f"{r['slow_aba_mean']:.2f}"] for n,r in zip(['补测基线','初版候选','最终候选'],online)]
phase_keys=[('感知筛选与变换','lidar_perception.pipeline/filter_transform_ms'),('分类体素','lidar_perception.pipeline/classification_voxel_ms'),('参考体素','lidar_perception.pipeline/reference_voxel_ms'),('地面分割','lidar_perception.pipeline/ground_ms'),('欧氏聚类','lidar_perception.pipeline/cluster_ms'),('真实射线键生成','lidar_mapping/integration/ray_keys_ms'),('叶节点更新','lidar_mapping/integration/tree_update_ms'),('全树内部占用刷新','lidar_mapping/integration/inner_occupancy_ms')]
phase_rows=[[label,f"{phase('baseline',k)['p95']:.3f}",f"{phase('optimized_final',k)['p95']:.3f}"] for label,k in phase_keys]
parts=[f'''# LiDAR 感知与建图性能、质量权衡（2026-10-09）

本轮已实现并实测：默认采用 OctoMap 原生逐路径更新和二维列缓存，保留全部真实端点、原积分频率、原分辨率和原地面门控。最终 1× 全程回放的建图采样累计 CPU 时间减少 **{pct(b['mapping_cpu_s'],f['mapping_cpu_s']):.1f}%**，积分 P95 **{b['integration_p95_ms']:.2f}→{f['integration_p95_ms']:.2f} ms**，投影与发布 P95 **{b['projection_publish_p95_ms']:.2f}→{f['projection_publish_p95_ms']:.2f} ms**。严格同输入的 445 张地图逐格一致；2591 帧保留真实强度的感知点云、包围框和门控也逐帧一致。

## 1 数据与测量口径

（1）完整阅读并保留上一轮 `OPTIMIZATION_2026-10-09.md`、`ld-6 地图质量优化.md` 和 `optimization_20261009/fast_final`。本轮补测基线沿用最终体素方案：分类 0.10 m、参考 0.05 m、全局 0.10 m、积分 2 Hz、发布 1 Hz，局部体素层 10 Hz。
（2）数据仍为 `bags/室外闭环`，2594 帧原始 LiDAR；基线、初版候选、最终队列候选三次 1× 全程回放串行执行，全部正常退出。每次复用 FAST-LIO 日志备份与恢复。旧报告、原始数据与 ld-6 笔记没有覆盖。
（3）`/proc` 每 0.5 s 采样各节点及启动器子进程：累计各 PID 的 CPU 时间并保留退出前采样最大值；同一时刻各进程 RSS 求和。**整链路 RSS 是采样峰值，不是不可漏采的内核高水位**。按节点提供 CPU/RSS，内部射线、叶更新、内部刷新、投影、体素、分割和聚类提供独立墙钟耗时，未把墙钟耗时冒充各函数 CPU 时间。
（4）445 帧建图核心对照使用同一批积分扫描 ID、真实原点与真实地面/障碍/清除终点；CPU 与 RSS 由单独进程 `getrusage` 记录，RSS 为内核高水位。所有候选均对这 445 个源时间输出快照；减半积分只处理 223 帧，另 222 帧保留旧图。CPU 包含输入读取、核心计算和快照文件写入，排除最终地图导出，不能与整链路的不同发布次数直接相减。
（5）世界系输入由保存的 TF 重建 float 变换，准备后的完整地图与 live 基线 PGM 有 **2 个边界格差异**；同一 `.bin` 输入在六个候选之间完全一致。core 与 live 分开解释。
（6）2591 帧感知对照保留真实强度，用相同点云和调平变换串行运行两个实现。旧的 462 帧 XYZ/强度零基准仍不用于强度相关分割结论。性能均为单次同机实测，无重复试验置信区间或跨机器/硬件外推。

## 2 实现改动与瓶颈

（1）基线全树内部占用刷新 P95 204.47 ms，而射线键生成仅 13.78 ms；全树投影/发布 P95 282.14 ms。默认启用原生非 lazy `updateNode`，让 OctoMap 按路径维护父节点和执行其原生裁剪，避开每帧全树 `updateInnerOccupancy`。
（2）二维缓存按 OctoMap 原生占用变更键维护细体素占用集合及列计数；同一 XY 的其他高度仍占用时不清除整列。完整投影作为独立对照入口，按粗叶节点的键范围展开整个二维覆盖。未知不补自由，地面支持仍按每帧去重、至少两次观测；无观测不删障碍。
（3）清除点云直接写入结果容器，预留输入容量；去掉聚类前重复建立搜索树。本机 PCL 1.14 的欧氏聚类器会自行建立一次树，全部几何阈值、分类体素和参考体素不变。没有把 0.05 m 参考体素链式重采样成 0.10 m 分类体素，因为代表点加权会改变原输入。
（4）初版候选在建图节点有 5 个缺地面话题的未配对帧，保留为失败证据。最终订阅缓存 5→20 帧，待配对容量 16→64 帧，超时仍为 1 s；增加实际接收计数及逐扫描缺失话题诊断。不能由一次复测证明所有负载下永远无缺失。

''',table(['内部阶段 P95（ms）','补测基线','最终候选'],phase_rows),f'''

清除输出阶段沿用诊断字段 `clearing_copy_ms`，最终实现没有执行整帧复制。内部阶段的计时边界独立，完整核心还包含分配、计时与结果整理；ROS 转换、TF 查询、排队与发布另计。内部耗时没有构造传感器到地图的因果端到端时延。

## 3 全程性能与接收核对

''',table(['指标','补测基线','最终候选'],[
 ['建图采样累计 CPU（s）',f"{b['mapping_cpu_s']:.2f}",f"{f['mapping_cpu_s']:.2f}"],
 ['建图采样峰值 RSS（MiB，含启动器）',f"{b['sampled_peak_rss_mib']:.1f}",f"{f['sampled_peak_rss_mib']:.1f}"],
 ['变换/积分 P95（ms）',f"{b['integration_p95_ms']:.2f}",f"{f['integration_p95_ms']:.2f}"],
 ['投影/发布 P95（ms）',f"{b['projection_publish_p95_ms']:.2f}",f"{f['projection_publish_p95_ms']:.2f}"],
 ['感知核心 P95（ms）','4.595','4.393'],['感知内部丢弃','0','0'],['局部输出 Hz','10.00','10.00'],['局部间隔 P95（ms）','100.46','100.47'],['OctoMap 节点','11429393','9113474']]),'''

最终 2591 帧已处理，参考有效 2379 帧（91.82%）；地面消息接收 2591 个，障碍/清除各 2379 个，447 帧积分＋1932 帧主动限频正好等于 2379。212 个未配对扫描均逐 ID 匹配 `GROUND_UNCERTAIN`，缺失掩码均为障碍＋清除；额外未配对、TF 丢弃、拒绝及尾部待配对均为 0。

补测基线参考有效 2380/2591（91.86%），观察器的 211 个缺失掩码均对应门控；建图累计只核到 210 个未配对和 2378 个完整帧，接收边界差额没有逐 ID 记录，不能独立定位。上一轮 224 个未配对与 222 个门控帧的旧差额也仍保留，不能追溯宣称已查清或全称通信丢包。raw/body 计数不同，不代表整条流水线零丢帧。

严格同感知输入：两个实现参考有效均为 2380/2591，所有 ground/obstacles/clearing/removed 与包围框文件 2591/2591 逐帧一致，统计字段除耗时外全部一致。平均核心 **2.490→2.363 ms（减少 5.1%）**，P95 **4.221→4.020 ms**。这证明保存输入上实现输出一致，没有语义标注准确率。

## 4 六个同输入建图候选

''',table(['实现 / 参数','积分帧','CPU（s）','峰值 RSS（MiB）','积分 P95（ms）','投影 P95（ms）','相同快照'],core_rows),f'''

默认原生路径更新＋缓存的核心 CPU 减少 61.9%，内核峰值 RSS 519.4→443.6 MiB。仅缓存虽然减少投影，却增加变更跟踪成本和内存，积分 P95 350.3 ms；独立每20帧裁剪的积分 P95 336.1 ms，也不设为默认。四个实现候选的 **445/445** 快照尺寸、原点和所有三态格完全相同；没有用更密输出美化质量指标。

![核心性能与自由空间]({root}/figures/core_tradeoffs.png)

''',table(['实现 / 参数','自由格','占用格','未知比例','最大原始自由分量格数','最终不同格'],geo_rows),'''

原始自由分量采用 4 邻接、只计自由格，没有膨胀，也不是可通行真值。射线2000预算使自由格减少 3.23%，占用格增加 16.80%；未知比例下降主要来自占用增加，不能解释成自由覆盖改善。积分减半使自由格减少 62.91%，最大原始自由分量 4128→431 格。两者均保持分类障碍/地面输入不变，只改变清除或累计积分证据。

''',f'![参数预算下的地图]({root}/figures/map_budget_comparison.png)',f'''

核心稳定性按共同约600 ms 源时间样本比较：{len(stable['source_stamps_ns'])} 张快照、315 对、39 个共同低速对。完整扫描、仅缓存、定期裁剪、默认原生更新的低速保持率均 99.47%、切换比例均 0.0738%、ABA 均 1.95 格。射线抽样与减半积分的保持率反而更高（99.67%/99.63%），但自由空间/固定路径退化；少清除和少输入可以让地图显得更稳定，不证明更准确。

## 5 独立 ROS 运行的几何与局部稳定性

`compare_optimization.py --profiles baseline optimized optimized_final` 沿用共同时间网格、最大70 ms 匹配误差与三组共同低速规则，得到414个地图对、40个共同低速对、38个共同低速三帧组合；窗口边缘裁掉2格。

''',table(['全程配置','参考有效率','低速占用保持率','低速切换比例','低速 ABA 均值'],local_rows),f'''

三个局部地图结果接近，没有证据称稳定性显著提升。最终全程图仍为770×993、0.10 m；最后发布快照自由38688、占用26294、未知91.50%。这不是漏检率、定位精度或全路线通行覆盖。

独立两次 ROS 回放只有3/2591个相同扫描 ID 的 body 点数组完全一致，2587个强度数组一致，4帧点数不同；同 ID 不等于同 LIO 输出或同 TF。2357个共同有效障碍帧的逐帧双向近邻 P95 再取 P95 为0.181 m，包围范围最大差的 P95 为0.679 m。几何框仅在双方门控有效且中心距离≤0.30 m时最优配对，897帧框数不同，基线/候选未匹配框累计1224/1220，匹配框最大尺寸差的 P95 为0.432 m。原始未约束强制配对数据保留在 `ros_geometry_initial_unbounded.json`，不能把不相关框强行配对后的大距离当位移。

因此不宣称整套 ROS 输出逐格或障碍几何无差异；输入与时序变化的来源没有进一步分离。严格实现等价结论只来自固定输入的2591帧感知及445帧地图对照。没有障碍标注，不能输出 Precision、Recall、IoU 或清除准确率。

## 6 官方规划与保留的限制

7种地图加载配置完成测量，系统、FAST-LIO和播放器顶层返回0，地图尺寸加载正确。射线/减半候选首次收尾服务受限，新建通信实例的日志出现socket权限错误，失败保留；测试改为复用现有Probe通信实例，并把收尾服务失败作为非零退出条件后，两组在`interface_ray_direct`/`interface_half_direct`重新验证成功。最终7组的管理服务返回0；主动SIGINT结束时部分子进程返回-2，不等同异常崩溃。每个 core 候选都调用 Nav2 `IsPathValid`；后5个候选还复用完整扫描地图的固定起终点请求，避免换选点掩盖退化。

（1）完整扫描基准：4个轨迹请求官方有效0/4（1条极短返回路径被拒绝，3次208）；选取已观测膨胀代价地图分量的9.92 m路径有效，22.62 m路径被官方拒绝。
（2）仅缓存、定期裁剪、默认原生更新：固定基准6个请求的错误码/官方有效性全部与完整扫描一致；445张逐格同图也得到实际服务复核。
（3）射线预算和减半积分：原9.92 m有效路径与原22.62 m请求均变成208，固定轨迹请求仍0/4。重新筛选各自分量可以取得2条官方有效路径（射线约9.42/19.28 m；减半约5.20/6.73 m），但这不能抵消固定请求退化。
（4）最终 ROS 全程地图：轨迹请求依然0/4；重新选择分量端点的9.10/19.06 m路径通过官方检查，有选点偏差。上一轮8.38 m通过、26.05 m拒绝的记录保持原样，不用本轮不同地图与选点覆盖它。

官方有效性不等于实车安全，没有控制闭环、回环或跨会话定位。默认地面证据层仍不替换体素层。

## 7 配置、代码与验证

默认 `src/lidar_mapping/config/mapping.yaml` 已启用：`incremental_inner_updates: true`、`incremental_projection: true`、`prune_interval: 0`、`max_ray_endpoints: 0`、`pairing_queue_depth: 20`、`max_pending_frames: 64`。几何阈值、2 Hz积分、1 Hz发布、0.10 m全局地图不变；参数预算仅保留为独立实验。

主要改动：

- `lidar_mapping/occupancy_mapper.hpp/.cpp`：原生路径更新、变更键二维缓存、粗叶展开、可选裁剪/真实端点预算和阶段诊断；完整对照入口仍可用。
- `lidar_mapping/mapping_node.cpp`：点云变换计时、配对容量配置、实际接收计数与逐扫描缺失掩码、发布时转交栅格容器。
- `lidar_perception/types.hpp`、`cloud_pipeline.cpp`、`cluster_extractor.cpp`、`perception_node.cpp`：独立阶段计时、容量预留、清除输出避免复制、去掉重复建树。
- `evaluation/map_quality`：资源采样、真实强度/完整建图输入捕获、固定输入工具、质量/稳定性/固定请求规划复核、CSV/JSON/图表和报告；评测不进入算法核心。

Release 编译通过；15项核心测试（感知7、地图8）与5项指标测试通过；粗叶覆盖、多高度列清除、40轮混合射线/命中时原生路径更新与完整投影等价均有回归。退出回归3/3通过，全部生命周期转换及退出码为0。首次退出复测因当前环境阻止 ROS CLI 守护进程连接，转换未执行，失败保留于 `shutdown_regression`；脚本改用 `--no-daemon` 后在 `shutdown_direct` 完整重测。没有把未执行转换、只有退出码0的首测冒充通过。

FAST-LIO2与Nav2原生核心没有改动，`local_costmap_main.cpp` 原退出生命周期修复保留。C++保持Allman与既定签名/注释规则。旧感知两个源码归档重新编译后，前120帧的600个输出文件与基线全部相同，验证复现工具没有覆盖生产源码。

## 8 复现与交付

```bash
cd /home/h/lidar_ws
source scripts/setup_lidar_nav.sh
colcon build --merge-install --packages-select lidar_perception lidar_mapping --cmake-args -DCMAKE_BUILD_TYPE=Release
python3 evaluation/map_quality/run_quality.py --config evaluation/map_quality/performance_optimized_final.yaml --bag bags/室外闭环 --output evaluation/map_quality/results/performance_retest --navigation
```

补测旧映射实现使用 `performance_baseline.yaml`，其映射配置已冻结在独立 `mapping_performance_baseline.yaml`，不再跟随生产默认配置。当前感知生产代码已优化；要复现旧感知，使用归档基准工具，不把再次运行生产感知叫作旧感知基线。

固定输入工具可在新目录构建和执行：

```bash
bash evaluation/map_quality/build_performance_tools.sh /tmp/lidar_performance_tools evaluation/map_quality/results/performance_20261009/source_baseline
python3 evaluation/map_quality/prepare_performance.py --run evaluation/map_quality/results/performance_retest --output /tmp/lidar_performance_tools/input
python3 evaluation/map_quality/run_mapping_sweep.py --root /tmp/lidar_performance_tools
```

构建脚本只把旧cloud/cluster实现直接链接进独立工具，其他未改模块使用当前库，适用于当前源码版本；若以后分割器/类型改变，需冻结对应版本后再复现。保存输入、配置、源码与工具指纹、节点/阶段资源CSV、逐帧几何与配对CSV、6组地图快照、7种配置的官方规划JSON/路径CSV及2组收尾复测、验证日志和必要图均在本报告同名性能目录。主要索引为 `performance.json`、`online_summary.csv`、`core_summary.csv`、`perception_paired.json`、`comparison/comparison.json`、`mapping_stability.json`、`official_planning.csv` 和 `suite_status.json`。

最终取舍：保留不改变固定输入输出的实现优化与经全程验证的配对缓存，拒绝会削弱覆盖和原有效路径的参数预算；没有证明分割或定位绝对精度、全路线导航可靠性、实车时延或跨会话能力。
''']
report=Path(__file__).with_name('PERFORMANCE_2026-10-09.md');report.write_text(''.join(parts),encoding='utf-8')
print('Report:',report.resolve())
