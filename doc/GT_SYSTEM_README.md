# FAST-LIO GT 轨迹采集与重放（统一 Humble 工作区）

适用于 `fastlio-mid360_space`；部署先读 [Jetson 指南](JETSON_DEPLOY.md)。
源工程的历史算法/实验说明保留在原电脑 `reference/doc/`，不作为现行启动指令。
此系统是基于 FAST-LIO 的轨迹估计和记录，不是经过外部真值设备验证的绝对真值。

## 1. 建图 / 定位 / 录包

在工作区根目录执行，脚本选择本机 Humble 或挂载此工作区的容器：

第一次部署或修改 C++ 后先单独执行 `bash scripts/build.sh`，编译无需 PCD。
下面命令是编译完成后的运行命令，地图与搜索参数不传给 build。

```bash
bash scripts/run.sh mapping map_name:=lab_a record_bag:=true
# 正常 Ctrl+C → pcd_map/<启动时间戳>_lab_a.pcd，并收尾录包
bash scripts/run.sh localization map_path:=pcd_map/实际地图.pcd record_bag:=true
```

RViz2 默认关闭；建图/定位/重放脚本追加 `--rviz` 可开启，`--no-rviz` 可明确关闭，
也支持原来的 `rviz:=true/false`。GUI 开关与录包开关相互独立。

`record_bag` 默认 **false**；实验需显式开启。默认录制
`/livox/lidar /livox/imu /Odometry` 到工作区 `bags/<mode>_<时间戳>/`，
可加 `extra_bag_topics:='/vln/decision_log'`。存储占用取决于点数/频率，应测量并留足空间。
不要沿用旧版“默认录 bag”“固定 scans.pcd”“YAML 选择地图”的流程。

建图地图位于 `pcd_map/`，手动 `/map_save` 和 Ctrl+C 更新同次会话文件。
定位必须显式传入 `map_path`，不自动选示例或最新地图；地图只读，拒绝 `/map_save`。
默认先在地图原点 3 m 内搜索初始位置和完整朝向，初始化必须保持静止。
等 `/localization/status` ready 且 `/tracking/status` tracking 后再录制；未定位或跟踪不健康的请求被拒绝。
初始化结果可读 `localization.matched_pose`（x,y,z,yaw(rad)）；见
[启动重定位](STARTUP_RELOCALIZATION.md)。手动模式 `relocalize:=false` 才使用 YAML
`localization.initial_pose` 与 `/initialpose` 直接播种。
运行模式和地图路径不是在线切换参数，切换场景需正常退出后重新启动。

建图/定位的 `preprocess.*`、点筛选与外参/协方差保持一致；变动标定或过滤方式后
重新检查已有地图适用性。采集时减少动态遮挡并保持一致的安装方式，地图质量由实测验收。
后方操作员过滤仅覆盖配置中的扇区/距离带，不能过滤所有行人。

## 2. 同一运行环境中的 GT 客户端

本机另开 Bash 终端：

```bash
cd /实际位置/fastlio-mid360_space
source scripts/setenv.bash
# Docker 场景先在宿主机 bash docker/run.sh，下面命令在容器内执行
ros2 run fast_lio gt_record.py readpose --name lab_01
ros2 run fast_lio gt_record.py start --source manual --instr lab_01 --note "operator A"
# 执行任务
ros2 run fast_lio gt_record.py stop
ros2 run fast_lio gt_record.py start --source vln --instr lab_01
ros2 run fast_lio gt_record.py stop
ros2 run fast_lio gt_record.py start --source baseline --instr lab_01
ros2 run fast_lio gt_record.py stop
```

每段任务一个 start / stop；CSV 从开始录制就写到工作区 `records/`，默认每 1 s
同步，stop/正常退出收尾；强制退出仍可能丢最近缓冲或留下损坏尾行。文件名含
序号、标签、时间戳及独占随机后缀，不覆盖旧段。可传 record_dir:=/其他目录。
Docker 工作区挂载持久化 records/；Git、部署 ZIP 和镜像上下文排除轨迹数据。
CSV 保留原 9 列并追加匹配点数/比例/残差，显示只保留最近 2000 点，不截断 CSV。
坏帧/丢定位不录制可信样本，因此时间轴可能有缺口；不得跨丢定位区间插值当真值。

起点姿态表可以放在当前工作目录 `start_poses.csv`，或用 `--poses-file`：

```csv
instruction_id,x_m,y_m,z_m,yaw_deg
lab_01,0.0,0.0,0.0,0.0
```

```bash
ros2 run fast_lio gt_record.py start --source vln --instr lab_01 --poses-file start_poses.csv
# 临时起点，命令单位是 degree（与 YAML 的 rad 不同）
ros2 run fast_lio gt_record.py start --source manual --instr lab_01 --pose 0 0 0 0
```

上述 `--pose` / `--poses-file` 和自动读取 `start_poses.csv` 的播种功能仅用于
`relocalize:=false` 手动模式。自动模式不使用起点姿态表，等待 ready 后直接录制。
手动模式客户端在录制前发布 `/initialpose`；也可用 RViz 2D Pose Estimate。
录制中拒绝位姿跳变，建图模式忽略 `/initialpose`。初始化误差容限与环境/点云有关，
不保证固定平移/角度范围内一定收敛；开始采集前检查配准扫描是否贴合参考地图。

服务包括 `/start_path_record`、`/stop_path_record`、`/map_save`（Trigger）。
节点 `/laser_mapping` 的参数 `record.sample_every_n=1` 表示每有效解算帧记录一次，
实际频率取决于输入和处理；`record.republish_saved=false` 可关闭旧轨迹持续重发。

## 3. 离线建图 / 定位重放

PC 或 Jetson 编译相同工程，在两个同一 ROS 运行环境的终端执行：

```bash
# 终端 A：不启动硬件 driver；使用模拟时钟
bash scripts/run.sh replay config_file:=mid360.yaml map_name:=lab_a_offline use_sim_time:=true
# 或定位重放（同样强制选择地图）
bash scripts/run.sh replay config_file:=mid360_localization.yaml \
  map_path:=pcd_map/实际地图.pcd use_sim_time:=true
# 终端 B：source scripts/setenv.bash 后播放原始传感器数据
ros2 bag play /实际路径/bag目录 --clock --topics /livox/lidar /livox/imu
```

上述两种终端 A 命令二选一。按需加 `--rate 0.5`，不是重放必然没有丢帧/漂移。
只播放原始传感器 topic，避免旧 `/Odometry` / `/tf` 与新解算结果混合。
建图重放走完后 Ctrl+C 保存地图；定位可在重放期间调用 start / stop 导出 CSV。
重放是顺序因果滤波，不是回环优化或全局平滑，不能自动修复参考地图误差。

GT CSV 的位姿时间戳来自 LiDAR 帧结束时间，与 `/Odometry` 一致；
必须核对传感器时间、系统/ROS 时钟及策略日志的时基，不能仅因“在同一 bag”就假定对齐。
重放显式 `use_sim_time:=true` 配 `--clock`；需要根据实验 episode 时间正确打标。
跨机策略数据可通过时间同步和同包录制辅助对齐，并用消息时间戳验证。

## 4. CSV 与后处理

CSV 带 `#` 元数据行，随后列为：

```csv
x,y,z,qx,qy,qz,qw,stamp_sec,stamp_nanosec,effective_points,match_ratio,mean_residual
```

`camera_init` 为建图/参考地图坐标系，位姿为 IMU 状态，不是机器人底盘中心或外部真值。
处理脚本依赖 numpy，可选 matplotlib 绘图：

```bash
ros2 run fast_lio gt_postprocess.py compare --ref manual_lab01.csv --test vln_lab01.csv \
  --goal 2 3 --success-radius 0.5 --plot lab01.png --json
ros2 run fast_lio gt_postprocess.py summary --pairs manual_lab01.csv,vln_lab01.csv
ros2 run fast_lio gt_postprocess.py interp --traj vln_lab01.csv --times cmds.csv --out poses.csv
```

也可直接 `python3 src/FAST_LIO/scripts/gt_postprocess.py ...`（不需要 ROS 节点）。
compare 包含终点距离、成功判定、路径长度比和轨迹距离；没有给 `--goal` 时用参考终点。
同一地图坐标系数据通常不做 `--align-origin`，避免掩盖真实起点/配准误差。
成功半径与汇总统计应按自己的实验协议设置，不直接等同于论文评价标准。
interp 的时间 CSV 需 `t_sec` 表头，`cmd` 可选；位姿线性/四元数插值，超出区间标记无效。

## 5. 运行检查

`/livox/lidar` 必须是 `livox_ros_driver2/msg/CustomMsg`，配置 `lidar_type=1`。
检查输入频率、`/Odometry` 时间滞后、CPU/内存和可用磁盘；长跑可关闭 RViz 和全图发布，
但不要为了性能单独改变建图/定位物理参数。参考图发布 `/reference_map`，RViz
使用 Transient Local QoS；世界系扫描是 `/cloud_registered`，Fixed Frame 为 `camera_init`。

当前电脑测试边界见 [验证记录](../VERIFICATION.md)：ARM64 原生编译、真实雷达精度
与 Jetson 实时性能仍须现场测试。历史文档中的破坏性 git 回滚指令不适用于无 Git 的部署包。
