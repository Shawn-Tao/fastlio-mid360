# 脚本入口

## 常用指令：NX 计算，AGX 看图

<!-- BEGIN QUICK_COMMANDS -->

所有命令均在各自机器的 `fastlio-mid360_space` 根目录执行。NX 负责接雷达和计算，
AGX 只负责显示；先准备可用的 Humble 环境和 NX 的本地雷达配置。首次部署步骤见下文。

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

日常只记 `build.sh`、`run.sh`、`rviz.sh`、`test.sh`。
`run.sh --rviz` 在计算端同时开 GUI；`rviz.sh` 只启动查看器，不能代替计算端。

## 顶层保留的 10 个入口

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
| 交互 | setenv.bash | Bash 中 source，加载 ROS / 可选 overlay / DDS |
| 交互 | setenv.zsh | Zsh 中 source，同上；与 CPU 架构无关 |

`lib/` 是这些入口使用的内部实现，不需要手动运行。
`backup/` 保存 5 个历史别名，可回看/手动使用，不进入 Docker 上下文或部署 ZIP。
没有删除历史脚本，也没有把必需 helper 当作备份。

## 环境选择和高级用法

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

脚本使用 Bash，因此 Zsh 终端也可直接 `bash scripts/run.sh ...`。只有交互 source
需按当前 shell 选择 setenv.bash / setenv.zsh。改变模式/地图不需重新编译，
但旧 install 副本使用本次更新的 launch/RViz 配置前应先重新 build。
