# 地图原点附近的启动重定位

## 常用指令：编译和启动是两步

第一步只编译代码，无需 PCD、搜索半径或 RViz2 参数：

```bash
bash scripts/build.sh
```

第二步在 NX 指定本地雷达配置、地图并启动定位：

```bash
bash scripts/run.sh localization \
  lidar_config:=config/local/MID360.jetson.local.json \
  map_path:=pcd_map/实际地图.pcd search_radius:=3.0
```

AGX 独立查看定位（已有 Humble、RViz2 和桌面，不需复制地图或编译工程）：

```bash
bash scripts/rviz.sh localization
```

所有命令在各自工作区根目录执行。NX 默认不开 RViz2、不录 bag；本机可视化
追加 `--rviz`，录包追加 `record_bag:=true`，明确关闭 GUI 可用 `--no-rviz`，
也兼容 `rviz:=true/false`。两端默认 Fast DDS/domain 18，环境覆盖须一致。
本地 JSON 首次创建与建图步骤见 [主文档](../README.md)。

换 PCD、搜索半径或 RViz2 开关不需要重新编译。代码块中的换行分隔两条命令；
只有行尾的 `\` 才把下一行接到同一条命令，不能在 `build.sh` 后接定位参数。

## 原理和适用范围

定位默认启用启动重定位，不再要求准确站回建图起点或复现建图朝向。
必须选择当前场地的实际地图；随包 `test.pcd` 只适用于程序启动自检。

默认搜索中心 `[0,0,0]` 是建图开始时的 IMU 原点，不是 PCD 点云质心，
也不是机器人 base_link 或当前扫描中心。搜索严格限制在中心的三维距离 3 m
以内，并额外限制高度偏差不超过 0.5 m；朝向覆盖完整 360°。
3 m 约束仅用于启动，初始化完成后的正常运动不受此范围限制。
如果地图经过外部平移/旋转或裁剪，不要假定原点仍是建图起点。

## 流程和使用约束

保持静止：IMU 初始化 → 累积 15 帧 → 后台粗搜索 → 候选精配准 →
用 3 帧新扫描复核 → 输出初始位姿并开始 FAST-LIO 持续跟踪。
匹配过程中 ROS 回调继续接收传感器，不阻塞在整张地图搜索中。

- 先将参考地图和 IMU 扫描重力对齐，再搜索 x/y/z/yaw；roll/pitch 由重力约束，
  不参与网格搜索。默认允许初始倾斜 20°，仍要求同一楼层、安装/标定一致，
  不支持翻转、任意 6-DOF 或跨楼层启动。结果转换回原地图坐标系，搜索高度沿重力方向。
- IMU 的角速度和加速度变化检查是运动拒绝启发式，不能证明设备静止；
  恒速平移也可能无法被 IMU 检测。操作者必须保持静止直到 `ready`。
- 粗搜索为位置/高度/朝向网格，多候选精配准使用带裁剪的四自由度点到点 ICP。
  不新增 Open3D、GPU、Scan Context 或学习模型依赖。
- 默认要求体素化后至少 120 个有效匹配点、扫描重叠率 ≥0.65、
  匹配点 RMSE ≤0.18 m；distinct 候选的截断 RMS 代价差 <0.02 m 时拒绝歧义。
  这些是初始调参值，不是对实机场景的成功率或定位精度保证。
- 默认最多精配准 24 个候选，搜索耗时上限 20 s。搜索未完成/超时，不接受部分结果。
  网格、候选数和上限影响覆盖与计算量；非全局最优算法，没有穷尽所有连续位姿。
- 走廊、对称房间、动态遮挡、地图质量不足仍可能导致失败或误匹配。
  启动验收前应叠加 `/reference_map` 与 `/cloud_registered` 做人工检查。
- `ready` 表示启动匹配已通过；开始录制还须 `/tracking/status` tracking。
  持续异常会更新 localization/status 为 degraded/lost，lost 须明确重试，不能偷偷跳位姿。
  门限不能保证导航安全或绝对真值。详见 [运行安全与验收](RUNTIME_SAFETY.md)。
  下游应监测状态和传感器/里程计新鲜度，不能仅依赖 ROS 话题存在。

## 状态、结果和重试

状态 `/localization/status` 为 transient-local `std_msgs/msg/String`，内容为 JSON：

```text
{"state":"waiting_for_imu","detail":"waiting_for_imu"}
{"state":"collecting","detail":"collecting_stationary_scans"}
{"state":"matching","detail":"searching_position_and_heading"}
{"state":"confirming","detail":"confirming_fresh_scans"}
{"state":"ready","detail":"matched_and_confirmed"}
```

失败为 `state=failed`，detail 常见 `poor_match`、`not_converged`、`ambiguous`、
`timeout`、`motion_during_search` 或 `fresh_scan_confirmation_failed`。
初始化未通过不发布 `/Odometry`、跟踪扫描或新轨迹，轨迹录制服务返回失败；
参考地图仍正常发布。不会自动把失败结果当作 `[0,0,0,0]` 开始跟踪。

在同一 ROS 环境和 domain 的终端：

```bash
source scripts/env.sh
ros2 topic echo /localization/status --qos-durability transient_local
ros2 param get /laser_mapping localization.matched_pose
ros2 param get /laser_mapping localization.matched_rmse
ros2 param get /laser_mapping localization.matched_overlap
ros2 topic echo /localization/initial_pose --qos-durability transient_local
```

`matched_pose` 是 `[x,y,z,yaw]`（米、弧度），搜索前/重试后为空。
`matched_rmse` 未匹配时为 -1，`matched_overlap` 为 0。参数仅更新内存，不修改 YAML。
结果话题为 `PoseWithCovarianceStamped`，frame=`camera_init`，位姿是 map←IMU。
其 covariance 是滤波器初始化策略值，不是实测统计置信度。重试后结果话题可能
保留上次 latched 位姿；必须以最新状态和结果参数判断有效性。
成功日志打印初始位姿、yaw(deg)、重叠率、RMSE 和搜索耗时。

保持静止，核对地图和范围后重试：

```bash
ros2 service call /relocalize std_srvs/srv/Trigger '{}'
```

服务 success 表示已发起请求，不表示匹配完成。搜索进行中返回 busy；
轨迹录制中拒绝重定位，须先停止录制。重试清除旧 ready 状态、结果参数和
传感器缓冲，并重新初始化 IMU。不会修改参考 PCD，也不会向参考地图加入点。

## 自定义范围与手动兼容模式

高级参数位于 `src/FAST_LIO/config/mid360_localization.yaml` 的
`localization.relocalization`。推荐复制为被 Git 忽略的本地配置：

```bash
mkdir -p config/local
cp src/FAST_LIO/config/mid360_localization.yaml config/local/scene.local.yaml
# 编辑本地 YAML 后显式选择，不改版本管理中的模板
bash scripts/run.sh localization map_path:=pcd_map/实际地图.pcd \
  config_file:=config/local/scene.local.yaml
```

`center` 可选择已知场景起点；`radius`、`z_range` 限制位置，
`xy_step`、`z_step`、`yaw_step_deg` 决定粗搜索网格。
增加搜索半径可能显著增加耗时；网格超过 100000 seeds 时启动拒绝。
搜索配置只在节点启动时读取；运行中 `ros2 param set` 不更新搜索模块。
改变范围/门限须重新启动，`/relocalize` 复用启动时的配置。
不要仅为“让程序成功”随意放宽匹配质量门限。

```bash
# 显式关闭自动匹配，恢复 YAML initial_pose 和 /initialpose 的直接播种
bash scripts/run.sh localization map_path:=pcd_map/实际地图.pcd relocalize:=false
```

手动模式状态为 `manual`，不提供自动匹配验证。自动模式忽略 `/initialpose` 的
直接位姿跳变，以免绕过质量门限。GT 客户端的 `--pose`/`--poses-file`
仅用于手动模式；自动模式应等待 ready 后直接录制，不用这些选项。
定位 replay 也使用同一启动匹配；原始 bag 开头需有静止段，匹配期间应暂停
运动段的播放，或者使用手动模式加已知初始位姿。

## 无 ROS 的算法回归

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
  -Isrc/FAST_LIO/include src/FAST_LIO/test/relocalization_test.cpp \
  -o /tmp/fastlio-relocalization-test
/tmp/fastlio-relocalization-test
python3 tests/test_relocalization_config.py
```

原生测试覆盖合成场景位置/完整朝向恢复、独立扫描复核、边界、错误场景、
歧义、取消、超时以及启动状态机。配置测试使用显式 ROS action stubs，
只验证参数流转，不代替 Humble 的 C++ 编译、ROS 启动或 MID360 实机精度验收。
完整 ROS 回归仍运行 `scripts/build.sh` 和 `scripts/test.sh`。
