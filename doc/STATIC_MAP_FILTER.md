# 建图存图层：时序静态点确认与自由空间清理

## 先用这些指令

更新源码后，在 NX 工作区根目录重新编译、自检，再建图。PCD 不是编译依赖。
本机雷达 JSON 使用已经创建且检查过的副本；以下名字仅为示例。

```bash
bash scripts/build.sh
bash scripts/test.sh
bash scripts/run.sh mapping lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=lab_a publish_map:=true record_bag:=true
```

更新本次源码须先编译一次；以后调整参数只需正常退出后重启，不需要重新编译。
两个阶段默认开启，已采用放宽组：入图 3 次/0.6 s、清理 3 次/0.4 s，
两者计票间隔 0.1 s，清理速度/转速上限 1.0 m/s、1.0 rad/s。
上面的普通建图指令直接使用新默认，不再需要显式传这八个参数。
这组仍须现场验证，降低入图门槛可能增加行人残留，降低清理门槛/放宽运动许可可能增加误删。
旧本地 YAML 和显式 CLI 优先，不会被自动改写。需要恢复**最初严格组**做对照时：

```bash
bash scripts/run.sh mapping lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=quad_strict_a publish_map:=true record_bag:=true \
  map_confirm:=true confirm_hits:=4 confirm_seconds:=1.2 confirm_interval:=0.2 \
  ray_clear:=true clear_hits:=6 clear_seconds:=1.0 clear_interval:=0.2 \
  clear_max_speed:=0.5 clear_max_angular_speed:=0.3
```

只测试入图确认：`map_confirm:=true ray_clear:=false`；只测试清理：
`map_confirm:=false ray_clear:=true`；两者都关闭：`map_confirm:=false ray_clear:=false`。
开关单独关闭不会连带关闭另一阶段；次数/跨度/间隔也可以分别覆盖。

默认开启存图过滤；默认不开 RViz、不录 bag，上例是主动录包。
AGX 独立看图仍用 `bash scripts/rviz.sh mapping`。NX 同一 ROS/Docker 环境另开终端：

```bash
source scripts/env.sh
ros2 topic echo /tracking/status --once --qos-durability transient_local
```

查看 JSON 的 `static_map`：`confirmed` 已确认存图点数、`candidates` 待确认点数、
`cleared` 累计清除数、`clear_skipped_frames` 未允许清理的帧数、`last_update_ms`
最近一次过滤耗时。`map_complete=false` 必须排查容量/帧输入截断，不能当完整图定位。
新增 `confirmation_enabled` / `clearing_enabled` 是实际生效开关，`confirmation_interval` /
`clear_interval` 是实际计票间隔。顶层 `archive_clear_gate` 是最近存图帧清理门控原因：
allowed、disabled、no_previous_frame、frame_points_truncated、frame_gap、position_std、
speed、pose_angular_speed、imu_peak_angular_speed、invalid_motion 等。
`clear_skipped_frames` 仅统计清理阶段已开启但未获许可的帧，不计有意关闭清理的帧。
可以用 `ros2 param dump /laser_mapping` 查看启动参数；PCD JSON 保存本次组合，便于对照。
正常 Ctrl+C 等待最终保存；PCD 和 `.pcd.json` 配对保管。

关闭过滤做对照（换会话名，不覆盖已有地图）：

```bash
bash scripts/run.sh mapping lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=lab_a_legacy static_filter:=false record_bag:=true
```

`static_filter:=auto` 使用 YAML，默认 true；true/false 可显式覆盖。
`map_confirm:=auto` / `ray_clear:=auto` 同样遵循 YAML。
总开关 static_filter=false 始终优先于子开关；显式要求开启/调参却关闭总开关会提前报错，
请加 static_filter:=true。两个子开关都关闭仍使用有界体素缓存（含单帧输入上限）；
static_filter:=false 则走原完整体素/原始追加路径，不启用新的单帧截断上限。
这些都是建图启动参数，不是编译参数，不支持运行时修改；定位及定位 replay 拒绝覆盖项。
定位仍是 `run.sh localization map_path:=...`，不会清理或写入参考 PCD。

## 作用范围与机制

这是保守的几何/时序过滤，不是人体识别，也不是完整的动态 SLAM。
只影响独立全局存图缓存，以及从该缓存提取的建图显示、检查点、`/map_save`、最终 PCD。
FAST-LIO 实时匹配、状态估计、ikd-Tree 插点/裁剪不变；不会解决所有动态遮挡导致的跟踪问题。
定位 `/reference_map`、实时注册扫描也不会被此策略修改。

新点按世界坐标的 0.1 m 体素进入候选缓存，每体素保留最靠近中心的真实测量点及其属性。
同一帧的重复点只计一次，跨帧支持必须间隔至少 0.1 s；默认至少 3 次观察，
且首次到本次观察跨度至少 0.6 s，才能进入已确认地图。两条件都须满足。
未再次观察超过 5 s 的**候选**过期；不强制把候选导出，退出也不提升候选。
因此建图刚开始不会立即显示/存出地图，快速经过或采样稀疏的细节也可能未获确认。
关闭 confirmation_enabled 后直接接纳新体素，不再等候选确认，但保留体素去重/容量限制；
clearing_enabled 可以继续开启。反过来关闭 clearing_enabled 只禁止射线删除，不影响确认。

已确认点永不因年龄、机器人离开或一帧没有看到而删除。
只有当前帧有效回波之前的射线自由段，才提供负证据：

- 收集完整帧的所有有效端点，再选择有限射线；射线遇到更近的当前回波就停止，
  即使那个回波没被选中投射，也会保护其后方。候选点同样能充当遮挡端点。
- 距回波端点留出 0.5 m，最多清理 15 m 内；已存真实点距自由射线须 ≤0.025 m，
  且其本体素及紧邻体素没有当前回波。这个窄通道是为减少边缘/薄物体误删。
- 每体素每 0.1 s 最多计一次负证据；至少 3 次、时间跨度至少 0.4 s 才删除。
  新的正观察会清零负证据；超过 3 s 没有新的负证据也重新计数。
- 仅可信建图帧参与累积；清理还要求位置标准差 ≤0.10 m、速度 ≤1.0 m/s、
  相邻位姿转速和本扫描 IMU 峰值转速都 ≤1.0 rad/s、相邻可信帧间隔 ≤0.5 s。
  第一帧不清理。协方差只是 KF 合理性门限，不是外部精度认证。

使用扫描末端 LiDAR 位姿作为射线原点，已应用 LiDAR→IMU 外参。
它并非每束激光发射时的原始原点，因此快走、快转或不确定时只做正观察，不清理。
没有回波的方向、被操作员扇区过滤掉的回波都不凭空产生“空气”证据。

## 参数与性能边界

高级参数在 `src/FAST_LIO/config/mid360.yaml` 的 `static_map` 段。
需要调参时先复制到 `config/local/mid360_mapping.local.yaml`，显式选择本地 YAML：

```bash
cp src/FAST_LIO/config/mid360.yaml config/local/mid360_mapping.local.yaml
# 编辑副本中的 static_map 段，再启动；不需要重新编译
bash scripts/run.sh mapping config_file:=config/local/mid360_mapping.local.yaml \
  lidar_config:=config/local/MID360.jetson.local.json map_name:=lab_a record_bag:=true
```

下表 YAML 字段均位于 static_map 下。CLI 留空遵循 YAML，CLI 优先且不改写配置文件。

| 启动项 | YAML 字段 | 当前默认/单位 |
| --- | --- | --- |
| static_filter / map_confirm / ray_clear | enabled / confirmation_enabled / clearing_enabled | true / true / true |
| confirm_hits / confirm_seconds / confirm_interval | min_observations / confirmation_seconds / observation_interval | 3 / 0.6 s / 0.1 s |
| candidate_ttl | candidate_ttl | 5 s |
| clear_hits / clear_seconds / clear_interval | clear_observations / clear_seconds / clear_observation_interval | 3 / 0.4 s / 0.1 s |
| clear_vote_ttl | clear_vote_ttl | 3 s |
| clear_range / clear_endpoint_margin / clear_ray_clearance | clear_max_range / endpoint_margin / ray_clearance | 15 / 0.5 / 0.025 m |
| clear_max_speed / clear_max_angular_speed | max_clear_speed / max_clear_angular_speed | 1.0 m/s / 1.0 rad/s |
| clear_max_position_std / clear_max_frame_gap | max_clear_position_std / max_clear_frame_gap | 0.10 m / 0.5 s |
| clear_max_rays / clear_max_ray_steps | max_rays / max_ray_steps | 256 / 600 |
| map_max_candidates / map_max_frame_points | max_candidates / max_frame_points | 250000 / 100000 |

confirm_interval 与 clear_interval 现在互不影响。旧本地 YAML 没有 clear_observation_interval
时，节点以 -1 自动继承原 observation_interval，保留旧版行为；新默认 YAML 显式设置 0.1 s。
也可手动 clear_interval:=-1 恢复继承。

次数允许 ≥1，跨度允许 ≥0，间隔允许 ≥0（0 表示每个不同帧可计票）；
即使间隔为 0，同帧重复点/多条射线仍最多计一次。confirm_hits:=1 confirm_seconds:=0
可以立即接纳新点；clear_hits:=1 clear_seconds:=0 可一次自由证据删除，属于激进诊断设置。
仅改小跨度不一定更快，理想最早条件还受 `(次数-1)×间隔` 限制；实际还取决于能否观察到。
YAML 中 double 字段用浮点值（例如 0.0），CLI 会按类型转换，整数次数不可写成 3.5。

candidate_ttl 和 clear_vote_ttl 必须为正；启用对应阶段时还须分别大于确认/清理跨度。
运动门限和射线几何/预算须为正，射线保护距离须小于清理范围，ray_clearance ≤体素尺寸/2。
参数非法或组合矛盾会在启动 driver 前报错；直接运行节点也保留 C++ 校验。

调参方向：入图更宽松可降低 confirm_hits/confirm_seconds/confirm_interval，或增大 candidate_ttl；
射线计票更宽松可降低 clear_hits/clear_seconds/clear_interval、增大 clear_vote_ttl；
行走时更多帧获许可可提高速度、转速、位置标准差和帧间隔门限。后三类改动都可能增加误删，
降低确认条件也会让短暂停留的人更容易入图。不要一次大幅放宽所有条件。
转速包含俯仰/横滚峰值，放宽并不会让末端射线原点变成逐束真实原点；当前实现仍未改变此近似。
先在同一 bag 上对照四种开关组合，再分别调入图/清理，最终选定值保存在本地 YAML。

确认地图仍受 `pcd_save.max_points=2000000` 限制，不暗中淘汰历史区域。
候选采用有界哈希和每候选一项的年龄链表，清理只走选中射线，不逐帧遍历整个历史地图。
默认每帧最多 153600 次射线体素访问；超过步数只停止该射线，不推断未访问处为空。
轮换均匀抽样的射线子集，避免固定只检查扫描中的少数方向。

确认点/候选/单帧端点任一容量截断都会告警，`complete=false` 会锁存到本次会话结束。
单帧端点被截断时整个帧**禁止清理**，因为被丢弃的回波可能是遮挡物。
候选正常过期和射线预算受限不是容量截断；`complete=true` 也不代表无动态点或完整场地覆盖。
开启过滤要求正体素分辨率和正存图容量；使用 `voxel_size=0` 或 `max_points=0`
对照旧机制，必须同时 `static_filter:=false`。

`last_update_ms` 只计过滤器，不含坐标变换、EKF、快照提取、PCD 写入。
快照仍为 O(已确认地图点数)，后台保存和显示存在额外内存，须实测整体 CPU/延迟/峰值 RSS。
增加射线预算能增加覆盖，也增加 CPU；缩小体素需要更多射线步数，清理距离可能受到步数限制。
清理开启时，缩小存图体素还须确保 ray_clearance ≤ voxel_size/2，否则启动拒绝该配置。

JSON 保存策略版本、开关、全部门限、候选/确认/清理/截断统计及最新可信传感器时间。
旧 schema=1 的定位验证保持兼容；存图策略不是定位预处理，故不强制要求定位 YAML
启用它。文件大小/点数/CRC 与物理参数的原有检查仍然执行。

## 不能保证的事情与验收

短暂停留不足确认条件的人通常不会入图；站定超过确认时间的人可能被当作静态物体。
离开后只有实际射线重复穿过旧位置才可清除。稀疏采样、抽样预算、门控、遮挡或无后方回波
都可能使清理很慢或无法完成；3 次/0.4 s 是证据门限，不是保证 0.4 s 内清理。
机器人不再回看、一直行走太快或区域长久被遮挡，不能保证消除残留。
建议人员离开后慢速回看/短暂停留，让后方真实结构获得重复观测。
仍可能丢失稀疏薄杆/边缘，不能用这份地图承担安全避障保证。

现场对照同一原始 bag（默认不录，需主动开启），检查：

1. 空场静止、慢走建图，墙/地面/细杆是否仍有足够几何细节；观察候选确认和资源走势。
2. 人横穿、走动、站定后离开；旧位置重新被照到时 cleared 是否增长，残留是否可接受。
3. 人挡在墙前时，墙不应因没看到而删除；人离开后墙的正观察应重置负证据。
4. 离开区域再回来，历史墙面保留；快转/低质量帧不应触发清理。
5. 故意调小缓存/射线预算，容量截断标记不完整，射线预算减小只降低清理覆盖。
6. 保存/退出时观察峰值内存和延迟，检查 PCD/JSON 配对，再用新图进行多起点定位验收。

空确认缓存不写空 PCD；若清理后为空而此前已有会话检查点，退出会明确报错提示该
检查点过时，保留文件但不能把它当作最终清理图。仅有最终 PCD 无法恢复逐帧可见性，
不能对旧 PCD 自动补做本时序过滤；离线重建应从原始 bag 重放：

```bash
# 终端 A，更新后的代码默认开启过滤
bash scripts/run.sh replay config_file:=mid360.yaml map_name:=lab_a_clean use_sim_time:=true
# 终端 B，同环境 source 后；替换实际 bag，结束后终端 A 正常 Ctrl+C
ros2 bag play bags/实际建图bag --clock
```

新的地图使用新会话 PCD/JSON，不沿用旧图 JSON；这不是回环优化或全局位姿修正。
主机策略/合成测试不代表 Jetson 性能，最新验证范围见 [VERIFICATION.md](../VERIFICATION.md)。
