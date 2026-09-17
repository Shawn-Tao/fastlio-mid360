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

默认开启存图过滤；默认不开 RViz、不录 bag，上例是主动录包。
AGX 独立看图仍用 `bash scripts/rviz.sh mapping`。NX 同一 ROS/Docker 环境另开终端：

```bash
source scripts/env.sh
ros2 topic echo /tracking/status --once --qos-durability transient_local
```

查看 JSON 的 `static_map`：`confirmed` 已确认存图点数、`candidates` 待确认点数、
`cleared` 累计清除数、`clear_skipped_frames` 未允许清理的帧数、`last_update_ms`
最近一次过滤耗时。`map_complete=false` 必须排查容量/帧输入截断，不能当完整图定位。
正常 Ctrl+C 等待最终保存；PCD 和 `.pcd.json` 配对保管。

关闭过滤做对照（换会话名，不覆盖已有地图）：

```bash
bash scripts/run.sh mapping lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=lab_a_legacy static_filter:=false record_bag:=true
```

`static_filter:=auto` 使用 YAML，默认 true；true/false 可显式覆盖。
这是建图启动参数，不是编译参数，不支持运行时修改；定位模式拒绝这个覆盖项。
定位仍是 `run.sh localization map_path:=...`，不会清理或写入参考 PCD。

## 作用范围与机制

这是保守的几何/时序过滤，不是人体识别，也不是完整的动态 SLAM。
只影响独立全局存图缓存，以及从该缓存提取的建图显示、检查点、`/map_save`、最终 PCD。
FAST-LIO 实时匹配、状态估计、ikd-Tree 插点/裁剪不变；不会解决所有动态遮挡导致的跟踪问题。
定位 `/reference_map`、实时注册扫描也不会被此策略修改。

新点按世界坐标的 0.1 m 体素进入候选缓存，每体素保留最靠近中心的真实测量点及其属性。
同一帧的重复点只计一次，跨帧支持必须间隔至少 0.2 s；默认至少 4 次观察，
且首次到本次观察跨度至少 1.2 s，才能进入已确认地图。两条件都须满足。
未再次观察超过 5 s 的**候选**过期；不强制把候选导出，退出也不提升候选。
因此建图刚开始不会立即显示/存出地图，快速经过或采样稀疏的细节也可能未获确认。

已确认点永不因年龄、机器人离开或一帧没有看到而删除。
只有当前帧有效回波之前的射线自由段，才提供负证据：

- 收集完整帧的所有有效端点，再选择有限射线；射线遇到更近的当前回波就停止，
  即使那个回波没被选中投射，也会保护其后方。候选点同样能充当遮挡端点。
- 距回波端点留出 0.5 m，最多清理 15 m 内；已存真实点距自由射线须 ≤0.025 m，
  且其本体素及紧邻体素没有当前回波。这个窄通道是为减少边缘/薄物体误删。
- 每体素每 0.2 s 最多计一次负证据；至少 6 次、时间跨度至少 1 s 才删除。
  新的正观察会清零负证据；超过 3 s 没有新的负证据也重新计数。
- 仅可信建图帧参与累积；清理还要求位置标准差 ≤0.10 m、速度 ≤0.5 m/s、
  相邻位姿转速和本扫描 IMU 峰值转速都 ≤0.3 rad/s、相邻可信帧间隔 ≤0.5 s。
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

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| min_observations / confirmation_seconds | 4 / 1.2 s | 新体素确认次数与最小时间跨度 |
| observation_interval / candidate_ttl | 0.2 / 5 s | 独立支持间隔 / 未再次看到的候选过期 |
| clear_observations / clear_seconds / clear_vote_ttl | 6 / 1 / 3 s | 删除次数、跨度、负证据最大间隔 |
| clear_max_range / endpoint_margin / ray_clearance | 15 / 0.5 / 0.025 m | 清理距离、端点保护、负证据窄通道 |
| max_candidates / max_frame_points | 250000 / 100000 | 候选上限 / 完整帧端点缓存上限 |
| max_rays / max_ray_steps | 256 / 600 | 每帧射线条数 / 单射线体素遍历上限 |
| max_clear_position_std / max_clear_speed / max_clear_angular_speed | 0.10 m / 0.5 m/s / 0.3 rad/s | 清理额外门控 |

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
缩小存图体素时还须确保 ray_clearance ≤ voxel_size/2，否则启动拒绝该配置。

JSON 保存策略版本、开关、全部门限、候选/确认/清理/截断统计及最新可信传感器时间。
旧 schema=1 的定位验证保持兼容；存图策略不是定位预处理，故不强制要求定位 YAML
启用它。文件大小/点数/CRC 与物理参数的原有检查仍然执行。

## 不能保证的事情与验收

短暂停留不足确认条件的人通常不会入图；站定超过确认时间的人可能被当作静态物体。
离开后只有实际射线重复穿过旧位置才可清除。稀疏采样、抽样预算、门控、遮挡或无后方回波
都可能使清理很慢或无法完成；6 次/1 s 是证据门限，不是保证 1 s 内清理。
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
