# 脚本入口

## 常用指令：NX 计算，AGX 看图

<!-- BEGIN QUICK_COMMANDS -->

所有命令均在各自机器的 `fastlio-mid360_space` 根目录执行。NX 负责接雷达和计算，
AGX 只负责显示；先准备可用的 Humble 环境和 NX 的本地雷达配置。首次部署步骤见下文。

**交互终端：加载环境、查看话题**（Bash/Zsh 通用，每个新终端执行一次）：

```bash
source scripts/env.sh
ros2 topic list
ros2 node list
```

ROS 在 Docker 中时，先用 `bash docker/run.sh` 进入对应容器，再执行以上命令。
必须用 `source`，不能用 `bash scripts/env.sh`；后者无法修改当前终端环境。

**NX：编译和自检**（编译不需要 PCD；首次部署或更新代码后执行）：

```bash
bash scripts/build.sh
bash scripts/test.sh
```

**NX：建图**：

```bash
bash scripts/check_network.sh config/local/MID360.jetson.local.json
bash scripts/run.sh mapping \
  lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=lab_a publish_map:=true
```

**AGX：显示建图**（已有 Humble、RViz2 和可用桌面）：

```bash
bash scripts/rviz.sh mapping
```

NX 建图结束按 Ctrl+C，等待最终保存和退出；保留 `pcd_map/` 中实际生成的
PCD 和配套 `.pcd.json`。不要同时启动建图、定位两套计算端。

**NX：定位**（替换实际地图名；初始化时保持静止）：

```bash
bash scripts/run.sh localization \
  lidar_config:=config/local/MID360.jetson.local.json \
  map_path:=pcd_map/实际地图.pcd search_radius:=3.0
```

**AGX：显示定位**（先退出建图查看器）：

```bash
bash scripts/rviz.sh localization
```

计算端默认不启动 RViz2、不录 rosbag；本机看图追加 `--rviz`，录包追加
`record_bag:=true`。建图远程看图必须传 `publish_map:=true`。AGX 无需编译本工程、
安装 Livox SDK 或复制 PCD，只需本项目脚本/显示配置及可用的 Humble/RViz2。
两端默认 Fast DDS、`ROS_DOMAIN_ID=18`；环境覆盖值须一致，跨机不能设置
`ROS_LOCALHOST_ONLY=1`。更换地图、搜索半径或 RViz 开关不需要重新编译。

<!-- END QUICK_COMMANDS -->

初次创建 NX 本地配置（替换真实 IP；只创建一次，不覆盖已有文件）：

```bash
bash scripts/init_local_config.sh --name jetson \
  --host-ip 192.168.123.18 --lidar-ip 192.168.123.114
```

`host-ip` 是 NX 接雷达的地址，不是 AGX 地址。完整首次部署见
[Jetson 指南](../doc/JETSON_DEPLOY.md)，本地副本说明见 [config/README.md](../config/README.md)。

日常只记 `build.sh`、`run.sh`、`rviz.sh`、`test.sh`；交互查看话题用 `source scripts/env.sh`。
`run.sh --rviz` 在计算端同时开 GUI；`rviz.sh` 只启动查看器，不能代替计算端。

## 顶层保留的 11 个入口

| 分类 | 文件 | 用途 |
|---|---|---|
| 日常 | build.sh | 一次编译 driver + FAST-LIO；不需要 PCD |
| 日常 | run.sh | 唯一计算端启动入口：mapping / localization / replay |
| 日常 | rviz.sh | 只开 RViz：mapping / localization；不启动 driver/FAST-LIO/录包 |
| 验收 | test.sh | colcon 测试和无硬件启动自检 |
| 配置 | init_local_config.sh | 创建本机 IP JSON 副本，拒绝覆盖；无需 ROS/Docker |
| 配置 | check_network.sh | 只读检查 JSON 接收地址和路由，不改网卡 |
| 部署 | install_deps.sh | 首次部署依赖；--print 只查看，--apply 明确安装 |
| 部署 | package.sh | 新 ZIP + SHA256，拒绝覆盖旧包 |
| 交互 | env.sh | 推荐统一入口：source，自动识别 Bash/Zsh，显示实际 domain/RMW |
| 交互 | setenv.bash | Bash 中 source，加载 ROS / 可选 overlay / DDS |
| 交互 | setenv.zsh | Zsh 中 source，同上；与 CPU 架构无关 |

`lib/` 是这些入口使用的内部实现，不需要手动运行。
`backup/` 保存 5 个历史别名，可回看/手动使用，不进入 Docker 上下文或部署 ZIP。
没有删除历史脚本，也没有把必需 helper 当作备份。

建图 `run.sh mapping` 默认启用独立存图层时序确认/自由空间清理；`static_filter:=false`
关闭对照（建图/replay 建图专用，定位参考图只读）。不用增加新的脚本，现行 run.sh
直接转发该参数。查看 /tracking/status 的 static_map 统计；站定行人仍可能进入地图，
离开后须慢速重访旧位置。门限与现场验收见 [静态存图过滤](../doc/STATIC_MAP_FILTER.md)。

## 环境选择和高级用法

`env.sh` 加载 Humble、可选的本工作区 install overlay 及与启动脚本相同的 DDS。
未编译时只加载 ROS/DDS 并提示，仍能查看标准话题；未安装 Humble 时明确报错，
不假装已完成配置。它只作用于当前终端，不修改 .bashrc/.zshrc、系统网卡或其他终端。
已有 `setenv.bash` / `setenv.zsh` 保持可用，运行入口仍使用这些底层脚本。

如需使用非默认 domain，在 **source 前** 设置，NX、AGX 和交互终端均须一致：

```bash
export ROS_DOMAIN_ID=23
source scripts/env.sh
ros2 daemon stop                     # 切换 domain/RMW 后清理旧 CLI daemon
ros2 topic list
```

定位预设已为 `/reference_map` 配置 Transient Local。不同网卡、组播限制和
防火墙等见 [DDS 说明](../dds_config/README.md)。不要在 AGX 执行 run.sh 来“只看图”。

`run.sh` / `rviz.sh` / `build.sh` / `test.sh` 自动选择本机 ROS 或挂载本工作区的
运行中容器；可设置 FASTLIO_NATIVE=1 / 0 锁定本机 / Docker。Docker 看图需要
已安装 RViz2、GUI-enabled 容器和显示授权，脚本不会自动创建容器或修改 xhost：

```bash
# 如需 GUI 镜像，在 docker/build.sh 前设置 FASTLIO_INSTALL_RVIZ=1
# 创建 GUI 容器前设置 FASTLIO_DOCKER_GUI=1，详见 docker/README.md
ROS_DOMAIN_ID=23 bash scripts/rviz.sh localization  # NX 也须使用 domain 23
bash scripts/rviz.sh localization --config src/FAST_LIO/rviz/localization.rviz
bash scripts/rviz.sh localization -- --ros-args -r __node:=agx_viewer
bash scripts/run.sh --help
bash scripts/rviz.sh --help
```

运行脚本使用 Bash，因此 Zsh 终端也可直接 `bash scripts/run.sh ...`。交互用
`source scripts/env.sh` 自动适配，旧 setenv 入口仍需按 shell 选择。改变模式/地图不需重新编译，
但旧 install 副本使用本次更新的 launch/RViz 配置前应先重新 build。
