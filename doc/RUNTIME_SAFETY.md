# 建图、持续定位和长期运行

## 1. 常用指令和推荐流程

以下在 NX 工作区根目录执行，已准备可用 Humble/容器和本地雷达配置。
首次部署/更新后编译与自检：

```bash
bash scripts/build.sh                 # 编译不需要 PCD
bash scripts/test.sh                  # 必须在可用 Humble 中运行完整回归
```

NX 终端 A：建图并主动开启原始录包，发布远程显示地图：

```bash
bash scripts/run.sh mapping lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=lab_a publish_map:=true record_bag:=true
```

NX 终端 B：进入相同 ROS 环境后检查状态、请求保存（Bash；Docker 先进入对应容器）：

```bash
source scripts/setenv.bash
ros2 topic echo /tracking/status --once --qos-durability transient_local
ros2 service call /map_save std_srvs/srv/Trigger '{}'
ros2 topic echo /map_save/status --qos-durability transient_local
```

终端 B 的最后一条持续显示保存状态；等待本次 saved，再按 Ctrl+C 结束该查看命令。
之后在终端 A 正常 Ctrl+C，等待最终保存和退出。建图产生
`pcd_map/<时间戳>_lab_a.pcd` 和同名 `.pcd.json`。定位时建议严格校验：

```bash
bash scripts/run.sh localization lidar_config:=config/local/MID360.jetson.local.json \
  map_path:=pcd_map/实际地图.pcd \
  search_radius:=3.0 map_metadata:=strict
```

AGX 显示分别用 `bash scripts/rviz.sh mapping` / `localization`，两者择一；
完整跨机流程见 [主文档](../README.md)。仍默认关闭计算端 RViz 和 rosbag，
以上建图例子主动开启录包。`strict` 要求配套 JSON；旧地图无 JSON 默认 auto
仅警告兼容，不等于已经验证。实机 IP 使用本地副本，不修改默认模板。

本次修改保留 FAST-LIO 的局部 ikd-Tree / 迭代滤波主体，增加质量门控和独立
全局存图缓存。没有闭环或位姿图全局优化，不能把轨迹估计当作外部绝对真值。
原生策略测试不是 Humble/ARM64 编译、实机精度和性能验收。

## 2. 局部窗口与可信帧

- 建图窗口默认 400 m，`mapping.det_range=100 m`。沿用的移动阈值是
  `1.5 * det_range`，必须满足 `cube_side_length > 3 * det_range`；launch 和
  节点均拒绝不匹配配置。`det_range` 是窗口移动参数，不等于通用距离过滤器。
- 正常跟踪默认至少 20 个有效平面匹配点、匹配比例 0.05、平均绝对平面残差
  ≤0.20 m；状态/协方差须有限，位置协方差标准差上限 2 m。
  这些是可配置初值，不是定位精度保证，也不能完全检测几何退化或误匹配。
  Odometry 协方差改为发布前填写当前帧，并按位置/姿态顺序和地图固定轴变换；
  它仍只是滤波器内部不确定度，不能充当外部精度验证。
- `/tracking/status` 提供 `waiting/tracking/degraded/lost`、匹配点数/比例、残差、
  最后传感器时间戳、存图点数/完整性；周期心跳。只有 `tracking` 帧会发布
  Odometry/TF、修改实时地图、累积存图和录制轨迹。初始化建树是特殊阶段。
- 一次坏帧立即进入 degraded，不插点/录制；连续 3 帧正常后恢复；连续 10 帧
  不正常，或已开始跟踪后 1 s 没有新同步帧，进入锁定的 lost。
  回放暂停/极低倍速也会触发墙钟超时，需在本地 YAML 调整 stale_seconds。
- 传感器队列默认限制雷达 30 帧、IMU 2000 条；溢出或时间戳回退清空同步队列，
  拒绝继续假定轨迹连续。没有输入时拒绝开启轨迹录制。
- 自动定位 lost：停止录制，保持静止，调用 `/relocalize`，等待启动匹配和 tracking。
  `relocalize:=false` 的手动模式须重启，不能通过 `/initialpose` 跨断流恢复 IMU 积分。
  建图 lost：保存已有可信地图/保留原始 bag，正常退出后重新建图；不偷偷重置原点。
  原始 driver/rosbag 可继续运行；下游必须处理位姿断流，不能一直使用旧位置。
- `/localization/status` 在持续跟踪异常时也会转为 degraded/lost；ready 仅说明
  启动匹配通过，开始采集前还须检查 `/tracking/status`。

除录制标签、sample_every_n 和节点内部匹配结果外，参数为启动配置；在线设置
会被拒绝。修改 `config/local/*.local.yaml` 并用 `config_file:=...` 重启，无需重新编译。

## 3. 独立全局体素地图

- `pcd_save.voxel_size=0.1`：每体素保留离中心最近的实际点，不跨墙角直接求质心；
  这是有损降采样，细杆/边缘仍可能丢失，须对照原始 bag 验收。
- `dense_input=true` 从去畸变扫描累积，独立于实时扫描 0.5 m 滤波和显示开关。
  可改 false 降低存图逐点处理量，但可能减少地图细节。
- `max_points=2000000` 控制缓存点数；达上限后拒绝新体素但保留历史区域，告警并
  标记 complete=false。不会静默淘汰旧区域。0 明确取消上限，须自行评估内存。
- `voxel_size=0` 可对照旧的逐帧追加机制，仍受容量上限约束。
- 静止重复扫描时点数趋于稳定；噪声、漂移、新区域和动态物体仍可能增加点数。
  complete 只表示没有因容量截断，并不保证场地覆盖、几何精度或无动态物体。
- 实时 ikd-Tree 不因保存分辨率调整而修改；地图裁剪也只跟随可信位姿。

## 4. 后台保存与元数据

`/map_save` 成功表示快照已入队，不表示磁盘已完成；观察 `/map_save/status` 的
saving/saved/failed。并发保存会被拒绝。默认每 60 s 自动检查点，0 关闭；正常
退出等待已有任务并保存最新快照。磁盘写入和 CRC 在后台，快照提取仍在 executor，
是 O(地图点数) 操作，且短时需要额外内存，必须在 Jetson 测量。

PCD 和 JSON 各自使用同目录临时文件、文件同步和原子 rename。两文件不是一个
事务，JSON 记录点数、字节数和 CRC32 以检测中断/错配。CRC32 不是安全签名。
JSON 还记录标定、时间/预处理参数、存图分辨率、容量统计、参考重力、版本；
不记录实机 IP。标定/时间/操作员过滤不兼容或 complete=false 时拒绝定位。

- `map_metadata:=auto`：存在 JSON 时校验，旧 PCD 无 JSON 时警告后兼容启动。
- `strict`：必须有合法配套 JSON，推荐用于正式采集。
- `ignore`：明确跳过元数据检查，仅用于人工确认过的旧图/诊断，不宣称已验证。

地图外部平移/旋转、裁剪或重新滤波后，不能沿用旧 JSON；需要重新制作元数据并
验收，尤其是地图原点和重力方向。文件命名用系统时间；开机检查 `date -Is`，
跨机器时间比较前同步时钟。断电最多只能依赖上次完成的检查点/录包，不能保证最新数据。

## 5. 重力对齐和显示

启动时将参考地图与 IMU 扫描各自旋转到重力水平坐标系，再搜索 xyz/yaw；结果
转换回原 camera_init，完整姿态包含重力约束的 roll/pitch。默认 IMU 初始倾斜
上限 20°，仍要求静止、同楼层、同安装标定，不是任意 6-DOF 定位。
高度范围按重力方向计算，位置 3 m 球形约束不变。旧图缺失元数据时默认参考
重力 `[0.0,0.0,-1.0]`，若建图起点倾斜，应在本地 YAML 设置 localization.map_gravity
（ROS double 数组，各项写浮点数）。
gravity_alignment=false 可对照旧行为。

全图发布默认关闭。需要时在本地 YAML 开启 publish.map_en；有订阅者才发布，
默认 5 s、0.5 m 显示滤波、最多 100000 点。定位 reference_map 同样使用独立
0.5 m / 100000 点显示副本，一次 latched 发布；匹配地图仍为 0.1 m 加载副本。
建图 `--rviz` 会默认开启这个有限显示副本；远程 PC 显示可传 publish_map:=true，
publish_map:=false 可明确关闭。显示分辨率不影响算法地图。

## 6. 轨迹持久化

CSV 默认放工作区 records/，可用 record_dir:=/其他目录。Docker 工作区 bind
mount 可持久化这个目录；Git、镜像上下文和部署 ZIP 排除轨迹数据。
开始即创建独占文件，每个可信样本追加，默认每 1 s fflush+fdatasync；正常停止/
退出同步关闭。强制退出可能丢最近一段缓冲，读取器可检测未收尾记录和损坏尾行。

保留原 9 列，追加 effective_points/match_ratio/mean_residual。显示轨迹最多
2000 点，最多缓存 3 段，旧轨迹重发默认关闭；CSV 不受显示点数上限影响。
lost/degraded 不追加可信位姿，CSV 时间轴可能有缺口。评估时检查缺口/中断，不能
跨失定位区间插值并当作可信连续轨迹。

## 7. Jetson 验收

1. 在目标机执行完整 build/test；记录版本、CPU/内存/磁盘、输入频率及处理延迟。
2. 静止扫描至少 30 min，检查体素点数/内存走势；容量告警时不把图当完整图使用。
3. 走完整场地返回起点，检查闭合误差、墙面厚度/重影和原始 bag，不预设精度。
4. 多个 ≤3 m 起点、不同朝向/小倾斜重复定位，统计成功、失败、误匹配及总时延。
5. 模拟断流/错误地图/时间戳回退，确认进入 lost、停止可信输出，重定位不跨录制跳变。
6. 保存期间检查输入延迟和峰值内存；检查 PCD/JSON 配对、周期检查点和异常退出 CSV。

全图闭环优化不是本次实现。长距离累计漂移明显时，需要额外闭环/离线全局优化，
降低 rosbag 回放速度不等于优化地图。
