# ZIP → Jetson 测试指南

本包是源码部署包，不是可直接运行的 ARM 二进制。
包含 driver、FAST-LIO、完整第三方源码、ARM64/x86-64 两套 SDK、DDS / Docker / 脚本、
PCD 示例与文档。排除原电脑的 `build/ install/ log/ bags/`、Git / Python 缓存、
旧 `maps/` 和历史参考；`pcd_map/` 中的场景地图保留。示例 `test.pcd` 不对应你的实际场景。
Git / 部署 ZIP 都不携带 `config/local/` 或 `*.local.*` 的机器配置，需要在 Jetson 上重新创建。

## 1. 解压与环境确认

将 `.zip` 和同名 `.zip.sha256` 一起拷到 Jetson，在二者所在目录执行：

```bash
sha256sum -c fastlio-mid360_space_jetson_YYYYMMDD_HHMMSS.zip.sha256
unzip fastlio-mid360_space_jetson_YYYYMMDD_HHMMSS.zip
cd fastlio-mid360_space
uname -m
cat /etc/os-release
```

文件名替换成实际交付的名字。解压到新目录，不要覆盖旧工程或混用旧编译缓存。
预期 Jetson 指令集 `aarch64`。用 Linux `unzip` 解压以保留脚本执行权限；
即使中转工具丢失权限，下面 `bash scripts/...` 入口也能执行，但 SDK 和源码仍须完整。

[ROS Humble 官方平台说明](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)
列出 Ubuntu 22.04 Jammy 的 amd64 / arm64 支持。
不要在其他 Ubuntu 版本强行混装 Jammy 的 ROS apt 包，也不要因部署本工程而改 JetPack。

## 2A. 本机已经安装 Humble

以下依赖安装仅适用于 Ubuntu 22.04，需目标机能访问 Ubuntu / ROS apt 仓库：

```bash
bash scripts/install_deps.sh --print   # 先查看，不改变系统
bash scripts/install_deps.sh --apply   # 确认后安装缺少的依赖，不做 apt upgrade
export FASTLIO_NATIVE=1
FASTLIO_BUILD_JOBS=2 bash scripts/build.sh
bash scripts/test.sh
source scripts/setenv.bash             # Zsh：source scripts/setenv.zsh
```

如尚未装 ROS，先按 Humble 的官方安装流程安装，或用下面 Docker 分支。
本机默认检测到 `/opt/ros/humble/setup.bash` 后就直接编译，不再强制要求原电脑容器。
`FASTLIO_NATIVE=1` 显式锁定本机，`=0` 锁定 Docker。
内存紧张时 `FASTLIO_BUILD_JOBS=1`；构建日志应明确显示
`target=aarch64`、`livox-sdk-arm`。无需改 CMake 或手动设置 SDK 的 LD_LIBRARY_PATH。

## 2B. 使用 Docker 隔离 ROS 环境

目标机已安装可用的 Docker 后：

```bash
bash docker/build.sh
bash docker/run.sh --detach
export FASTLIO_NATIVE=0
bash scripts/build.sh
bash scripts/test.sh
```

此 CPU-only 镜像支持 ARM64，默认不绑定 NVIDIA GPU runtime，详见
[Docker 说明](../docker/README.md)。下载镜像和 apt 依赖需要网络；ZIP 不包含镜像，
也不是完全离线安装包。ARM64 原生镜像构建和实际编译须在 Jetson 上验证。

## 3. 实机网络与建图

默认模板 `src/livox_ros_driver2/config/MID360.json` 保持版本化，不修改它的实机 IP。原配置雷达
`192.168.123.114`，接收主机 `192.168.123.18`，仅作当前工程配置，不保证适合目标机。
`host_ip` 必须是 Jetson/容器真实存在的接收网卡 IPv4；两处 `lidar_ip` 保持实际雷达地址一致。

```bash
ip -br addr
# 替换示例 IP；生成本机副本，不改系统网络/默认 JSON、不覆盖已有文件
bash scripts/init_local_config.sh --name jetson --host-ip 192.168.123.18 --lidar-ip 192.168.123.114
bash scripts/check_network.sh config/local/MID360.jetson.local.json
bash scripts/run.sh mapping lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=lab_a rviz:=false record_bag:=true
```

网络检查只读，不给网卡写固定地址、不修改路由或防火墙；旧
`config_interface.sh` 也改为只读检查。通过目标系统的网卡管理工具配置雷达网卡，
避免改动远程 SSH 使用的网卡。网络检查通过仍不代表收到雷达数据。

另开同一运行环境终端，source Humble / 本工作区后检查 `/livox/lidar`、`/livox/imu`、
`/Odometry` 的消息频率。Docker 场景先 `bash docker/run.sh`，再 source 环境。
静止完成 IMU 初始化后再移动；观察 CPU/内存和地图质量，不宣称固定精度。

地图默认放到 `pcd_map/启动时间戳_lab_a.pcd`，实际完整路径见启动日志。
`ros2 service call /map_save std_srvs/srv/Trigger '{}'` 可保存当前快照；正常 Ctrl+C
保存最终地图到同一文件。强制 kill、断电或 Ctrl+C 保存尚未完成时没有自动保存保证。
没有有效点云不创建空 PCD。长时间建图逐帧累积会增加内存，按场景分段。

## 4. 多场景定位与 PC 可视化

```bash
# 换成实际地图名；不传 map_path 会在启动 driver / 节点前报错
bash scripts/run.sh localization lidar_config:=config/local/MID360.jetson.local.json \
  map_path:=pcd_map/实际地图.pcd rviz:=false record_bag:=true
```

必须给出本运行环境可读的实际 `.pcd` 路径。脚本固定 CWD 为工作区根目录，
可用上述相对路径；Docker 中的绝对路径必须是容器路径。
定位只读地图，不保存建图 PCD。设置该场景对应的 `localization.initial_pose`
（x,y,z,yaw，yaw 单位 rad）；可通过 `/initialpose` 更新，不能把换地图当成自动重定位。
不要同时启动建图与定位两个 driver。

建议 Jetson 不开 RViz，PC 上使用同一 `ROS_DOMAIN_ID`，按
[DDS 说明](../dds_config/README.md) 选择 RMW、通信网卡及可选 peers。
雷达 UDP 收包与 PC↔Jetson DDS 是两个独立问题。
PC RViz 使用 `camera_init` Fixed Frame；参考地图订阅 `/reference_map`，
Durability 设 Transient Local，扫描看 `/cloud_registered`。
GT 轨迹采集与 bag 重放见 [GT 文档](GT_SYSTEM_README.md)。

## 5. 验收和重新打包

- 确认 ARM64 SDK 被选择、一次编译完成两个包。
- `scripts/test.sh` 通过后再接雷达；自检仅覆盖无硬件链路，不证明实时精度。
- 确认传感器消息、里程计、人工 `/map_save` 和 Ctrl+C 最终 PCD；用本场景地图定位。
- 跨机看图时检查 DDS / QoS；单机正常而 PC 无话题通常属于网络或发现问题。
- 本电脑的验证和未完成项见 [VERIFICATION.md](../VERIFICATION.md)。

```bash
bash scripts/package.sh               # 生成工作区外的新 ZIP + SHA256
```

打包不覆盖旧 ZIP，保留 `pcd_map` 场景地图。`~/Record_Path` 是 GT CSV 输出位置，
不在工作区内；Docker 中属于容器用户家目录，删除容器前请单独拷出。
`bags/` 也必须单独归档，部署 ZIP 默认不携带实验录包。
本地配置支持副本和相对路径，详见 [config/README.md](../config/README.md)；
Git 忽略规则、源码 LF 与二进制约定见 [版本管理说明](VERSION_CONTROL.md)。
