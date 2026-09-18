# FAST-LIO + Livox MID360：ROS 2 Humble 统一工作区

## 1. 常用指令：NX 计算，AGX 看图

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
  publish_map:=true \
  map_name:=lab_a 
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
  search_radius:=3.0 \
  map_path:=pcd_map/实际地图.pcd 
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

### 首次使用：创建 NX 本地雷达配置

只需创建一次；将示例 IP 换成 NX 接收网卡和雷达的真实地址，**不要填 AGX 的 IP**：

```bash
bash scripts/init_local_config.sh --name jetson \
  --host-ip 192.168.123.18 --lidar-ip 192.168.123.114
```

生成 `config/local/MID360.jetson.local.json`，拒绝覆盖已有文件；以后只编辑本地副本，
不改默认模板。这个创建命令不需要 ROS/Docker/sudo，也不会配置系统网卡。
Git 和部署 ZIP 不携带本地配置，换机器须重新创建。

首次部署见 [Jetson 指南](doc/JETSON_DEPLOY.md)；
入口清单见 [脚本说明](scripts/README.md)；跨机通信见 [DDS 说明](dds_config/README.md)。
本机已有 Humble 时脚本默认本机运行；否则查找挂载本工作区的运行中容器，
不会自动创建容器。指定 `FASTLIO_CONTAINER` 或 `FASTLIO_NATIVE=0` 可锁定 Docker。
`test.sh` 是无雷达自检，不代表实机精度验收。

### 手动编译和环境选择

driver 与 FAST-LIO 位于同一 `src/` 下，只需一次 `colcon build --symlink-install`。
colcon 依据包依赖先编译 driver、再编译 FAST-LIO，不需要中途 source driver，
也不需要预先安装 SDK 到 `/usr/local`。

在**容器内的工作区根目录**手动编译、启动：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source scripts/setenv.bash
ros2 launch fast_lio mid360.launch.py mode:=mapping map_name:=lab_a rviz:=false \
  lidar_config:=config/local/MID360.jetson.local.json
```

`local_setup.bash` 在已 source Humble 后加载本工作区；也可直接 `source install/setup.bash`。使用 zsh 时相应换成 `setup.zsh` / `local_setup.zsh`。不要混入旧 driver / FAST-LIO 工作区的 overlay。环境入口不分 x86/ARM，旧架构别名已归档到 scripts/backup/。

交互 ROS CLI 推荐 `source scripts/env.sh`，自动适配 Bash/Zsh，并打印实际工作区、
domain 和 RMW；旧 `setenv.bash` / `setenv.zsh` 仍保留。默认 DDS 域 18，已有
`ROS_DOMAIN_ID`/RMW 覆盖值会保留。仅 source ROS/install 不会设置 domain。
PC、Jetson 和另开终端必须保持同一个 `ROS_DOMAIN_ID`；修改域后先 `ros2 daemon stop`。

低内存机器建议 `bash scripts/build.sh`：默认两个编译任务、包级串行，不删除任何已有产物。手动编译可用：

```bash
MAKEFLAGS='-j2 -l2' colcon build --symlink-install
```

脚本配置：

```bash
FASTLIO_CONTAINER=实际容器名或ID bash scripts/build.sh
FASTLIO_BUILD_JOBS=1 bash scripts/build.sh
# bind mount 自动解析不适用时，手动指定容器内工作区路径：
FASTLIO_CONTAINER_WORKSPACE=/其他路径/fastlio-mid360_space bash scripts/test.sh
# 不用 Docker，直接在安装了 Humble 的本机运行：
FASTLIO_NATIVE=1 bash scripts/build.sh
# 强制 Docker（多个匹配容器时还要指定 FASTLIO_CONTAINER）：
FASTLIO_NATIVE=0 bash scripts/build.sh
# 迁移到尚无 Humble 环境的机器时，首次创建容器：
bash docker/build.sh
bash docker/run.sh --detach
```

## 2. 目录与复制范围

原电脑目录：`/home/shawntao/workspace/humble_space/fastlio-mid360_space`。
原验证容器：`3018b9a5759e`，挂载目录：`/workspace/humble_space/fastlio-mid360_space`。
部署可以放在任意路径；脚本不固定原容器 ID，按工作区 bind mount 查找。
脚本顶层保留 11 个入口（新增统一交互环境入口 `env.sh`），内部实现放 `scripts/lib/`，5 个历史别名放
`scripts/backup/`；现行入口不依赖备份，部署 ZIP 不含备份目录。

```text
fastlio-mid360_space/
├── src/
│   ├── livox_ros_driver2/    # ROS2 Humble driver、消息、配置、第三方头文件
│   └── FAST_LIO/            # fast_lio 包，含完整 ikd-Tree / IKFoM 源码与 GT 工具
├── livox-sdk-x86/           # x86-64 预编译 SDK：include/ + lib/
├── livox-sdk-arm/           # ARM64 / aarch64 预编译 SDK：include/ + lib/
├── pcd_map/                 # 与 src/ 同级的统一地图目录
│   └── test.pcd             # 显式选择的自检示例，不作为默认定位地图
├── records/                 # 按需生成的持久化 CSV，不进入 Git/部署 ZIP
├── scripts/                # 11 个入口；lib/ 内部实现、backup/ 历史别名
├── config/                 # 本地配置初始化说明；local/ 不进入 Git 或部署包
├── dds_config/             # 可选 Cyclone DDS；不固定网卡、peer 或 domain
├── docker/                 # PC / Jetson 通用 Humble CPU 镜像与启动脚本
├── doc/                    # Jetson 部署、GT 记录与重放现行说明
├── tests/                  # 配置契约检查、无硬件启动测试、回环网络 JSON
├── reference/              # 原文件历史参考，不执行且不进入部署 ZIP
├── colcon.meta             # 为两个包开启本工作区的测试
└── build/ install/ log/    # 在目标机器生成，不应复制到另一架构复用
```

两个源工程按现有文件复制，保留许可证、算法改动、launch/config、第三方源码和 GT 记录工具；不复制 `.git`（包括指向旧仓库的子模块 `.git` 文件）、Python 缓存和旧工作区的 build/install/log。SDK 的头文件、共享库和静态库均完整复制。`reference/` 与 SDK 根目录带 `COLCON_IGNORE`，不会参与包发现。

原 `fastlio-space` 未修改。原 `livox_space` 的 driver 只修改了 CMake 的默认 ROS2/Humble 配置与 SDK 查找，并新增 `cmake/livox_sdk.cmake`；旧源码、SDK 和编译产物均保留。新工作区的完整配置与原工程独立，不引用原工程的源码或 SDK。现行 DDS、Docker、文档和脚本均以本工作区为准；历史脚本名仅在 `scripts/backup/` 中保留。

新副本还修正了 FAST-LIO 的退出处理：不再覆盖 rclcpp 自带的异步 SIGINT handler，避免在原 POSIX signal handler 内调用 shutdown() 导致 Ctrl+C 退出卡住。原有建图/持续跟踪主体保留；定位新增原点附近的启动重定位，详见 [启动重定位说明](doc/STARTUP_RELOCALIZATION.md)。

地图管理已统一：可信建图帧进入独立全局体素缓存，不依赖地图发布，也不从裁剪后的实时树直接导出完整地图；`/map_save`、周期检查点与正常 Ctrl+C 保存同次会话。旧 `maps/` 示例不是输出目录。长期运行和数据可信度说明见 [运行安全与验收](doc/RUNTIME_SAFETY.md)。

## 3. 原 driver 编译流程与问题

原入口是在容器中：

```bash
cd /workspace/humble_space/livox_space/livox_ros_space/src/livox_ros_driver2
./build.sh humble
```

该脚本的实际过程：

1. 解析 `humble`，设置 ROS2 与 Humble。
2. 删除 driver 工作区的 `build/`、`devel/`、`install/` 和可能存在的 `src/CMakeLists.txt`。
3. 用 `package_ROS2.xml` 覆盖 `package.xml`，临时复制 `launch_ROS2/` 到 `launch/`。
4. 回到 `livox_ros_space/`，调用 `colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble`。
5. CMake 生成 `CustomPoint` / `CustomMsg` 接口，链接 SDK，编译 ROS2 组件库及 `livox_ros_driver2_node`，安装配置与 `launch_ROS2/`。
6. 脚本移除临时 `launch/`。使用者 source driver 的 install 环境，再进入另一个工作区编译 FAST-LIO。

发现的问题：ROS2 查库目录写成不存在的 `livox_space-x86/livox-sdk`；ROS1 头文件路径还有 `o/workspace` 拼写错误；系统默认目录可能掩盖这些路径错误；Humble 消息 typesupport 分支依赖脚本传入参数；删除整个 install 目录不适合合并工作区。

新 driver 固定使用现有 ROS2 `package.xml`，CMake 直接调用 Humble 的 `rosidl_get_typesupport_target()`；配置和 launch 直接安装，不再临时复制/删除。新 driver 的 `./build.sh humble` 仅为兼容入口，转交统一工作区的编译脚本，不删除产物。原工程的旧 `build.sh` 保留，仍有上述清理行为，不要把它复制回来覆盖新脚本。

## 4. SDK 架构自动选择

逻辑在 `src/livox_ros_driver2/cmake/livox_sdk.cmake`，使用 CMake **目标架构** `CMAKE_SYSTEM_PROCESSOR`，不使用宿主机硬编码路径。

| 目标架构 | 自动选择 |
|---|---|
| `x86_64` / `amd64`（大小写不敏感） | 工作区根目录 `livox-sdk-x86` |
| `aarch64` / `arm64`（大小写不敏感） | 工作区根目录 `livox-sdk-arm` |
| 32 位 x86/ARM 或其他架构 | 明确报错：现有 SDK 不是这些架构 |

优先选择共享库，缺少时可选择静态库；查找限定在选中的 SDK 内，不回退 `/usr/local`。每次配置重做库查找，避免旧缓存继续指向另一个架构的 SDK；对于共享库还读取 ELF 头检查指令集，错误 override 会在配置阶段失败。共享库随 driver 安装到 `install/livox_ros_driver2/lib/`，source install 后即可找到，不需要手工设置 SDK `LD_LIBRARY_PATH`。

查看实际选择：编译日志中会出现 `Livox SDK: target=..., root=..., library=...`。

SDK 不在默认布局时：

```bash
colcon build --symlink-install --cmake-args -DLIVOX_SDK_ROOT=/绝对路径/sdk
# 恢复自动选择（覆盖之前缓存的手动值）：
colcon build --symlink-install --cmake-args -DLIVOX_SDK_ROOT=
```

复制到 ARM64 机器时，拷贝整个工作区但排除 `build/`、`install/`、`log/` 和 `bags/`，在 ARM64 的 Humble 环境重新编译。自动选库不等于交叉编译：交叉编译还需要 ARM64 工具链、sysroot 和 ARM64 ROS 依赖；此处只验证了 ARM64 SDK 的架构及查找逻辑，不宣称已经完成 ARM64 原生编译。

## 5. 实机网络配置

新工作区保留原 `MID360.json`：LiDAR `192.168.123.114`，接收端 `192.168.123.18`。它与另一个 `MID360_config.json` 示例不是同一套地址；默认 launch 使用的是 **`MID360.json`**。

默认 JSON 保持版本化、不直接编辑 IP。实机运行前从默认文件生成本机副本：

```bash
# 只有配置创建无需 ROS / Docker / sudo；示例 IP 请换成真实地址
bash scripts/init_local_config.sh --name jetson \
  --host-ip 192.168.123.18 --lidar-ip 192.168.123.114
bash scripts/check_network.sh config/local/MID360.jetson.local.json
bash scripts/run.sh mapping lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=lab_a publish_map:=true
```

不传 `--name` 时生成 `config/local/MID360.local.json`；不传 IP 时仅复制默认地址，
仍须按实机核对。初始化拒绝覆盖已有副本，不改变模板或系统网络。
脚本会联动更新以下 IP，其他参数保持不变；已有副本可直接编辑：

- `MID360.host_net_info[0].host_ip`：容器中真实存在、能够与雷达通信的网卡 IPv4 地址。
- `MID360.host_net_info[0].lidar_ip` 和 `lidar_configs[0].ip`：实际雷达 IP，两处保持一致。
- 保留现有 MID360 端口配置，确保 UDP 接收与防火墙设置匹配。

检查和启动必须选择同一份副本；相对路径按工作区根目录解析，容器通过 bind mount 访问。
不传 `lidar_config` 时仍用默认 JSON，不自动加载本地文件。本地配置被 Git、Docker
上下文和部署 ZIP 排除；部署到 Jetson 后重新生成。详见 [config/README.md](config/README.md)。

运行 `bash scripts/check_network.sh` 可检查默认 JSON，也可显式传本地 JSON 路径。该检查不设置网卡、不改变路由、不关闭防火墙，也不证明已经收到了雷达数据。

当前容器已使用 `network=host`，但其网络命名空间中没有 `192.168.123.18`。不要仅因为设置了 host 网络就假定 Docker Desktop/WSL 中已拥有物理雷达网卡；需要让容器实际拥有对应地址并能收发雷达 UDP，再开展实机测试。本次没有替你修改主机网络。

可通过启动参数选择其他配置文件：

```bash
bash scripts/run.sh mapping lidar_config:=/容器内路径/MID360.json
# 仅启动 driver，在已经 source 环境的容器终端中：
ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  user_config_path:=/容器内路径/MID360.json
```

## 6. 建图、定位与离线重放

所有启动脚本支持 `--rviz` / `--no-rviz`，默认不启动 RViz2；原来的
`rviz:=true` / `rviz:=false` 保持兼容，多次指定时最后一个选择生效。
`--help` 可在未安装 ROS / 未启动 Docker 时查看帮助。
直接使用 `ros2 launch` 时使用 `rviz:=true/false`，不使用脚本的 `--rviz` 选项。

### 建图

```bash
bash scripts/run.sh mapping map_name:=lab_a
# 按需开启可视化和录包：
bash scripts/run.sh mapping map_name:=lab_a --rviz record_bag:=true
```

默认 `xfer_format=1`（`livox_ros_driver2/msg/CustomMsg`）、单雷达统一 topic、10 Hz。FAST-LIO 订阅 `/livox/lidar` 与 `/livox/imu`，输出 `/Odometry`、`/path` 等。不能把 driver 改为 `PointCloud2` 而仍沿用 `preprocess.lidar_type=1`。

在同一运行环境另开终端，进入工作区并 source 环境后检查（Docker 时先进入容器）：

```bash
source scripts/setenv.bash
ros2 node list
ros2 topic info /livox/lidar --verbose
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic echo /tracking/status
ros2 service call /map_save std_srvs/srv/Trigger '{}'
ros2 topic echo /map_save/status
```

地图统一保存在与 `src/` 同级的 **`pcd_map/`** 下，默认命名为：

```text
pcd_map/20260916_160000_123456_lab_a.pcd
        启动日期_启动时间_微秒_自定义场景名.pcd
```

`map_name` 默认 `map`，可以使用中文、字母、数字、下划线和连字符，不含 `.pcd` 后缀或路径分隔符。时间戳在节点启动时生成，使用所在容器/系统的时区。同次运行中，`/map_save` 会更新当前会话文件，正常 Ctrl+C 则把最终地图保存到同一文件；不同运行默认使用不同时间戳，不覆盖之前的场景地图。启动日志会打印完整的 `Mapping session output` 路径。

`map_dir:=/其他目录` 可以改输出目录；`map_output:=/完整路径/new_map.pcd` 指定新会话路径，拒绝启动时已存在的目标。PCD 和配套 `.pcd.json` 各自临时写入、同步和原子替换。`/map_save` 成功仅表示快照已入队，必须查看 `/map_save/status` 的 saved/failed；同时只允许一个后台保存任务。默认每 60 s 检查点，正常退出保存最新快照；空地图不生成文件。

无已确认建图点时，`/map_save` 返回 `No mapping points available` 并提示待确认候选数，Ctrl+C 不创建空 PCD。刚开始时可信里程计可能已输出，但地图尚在确认；不会退出时把候选强行存入。无论 `publish.map_en` 或扫描显示开关是否开启，只要 `pcd_save.pcd_save_en=true` 就会累积建图结果。

`record_bag` 默认 false，主动开启后保存到 `bags/<mode>_<时间戳>/`；可用 `bag_dir` 和 `extra_bag_topics` 配置。每会话一份地图，不使用 `pcd_save.interval` 分片。存图默认从去畸变扫描输入、0.1 m 全局体素去重，独立于实时匹配 0.5 m 滤波；默认最多 2000000 点，容量不足会告警并标记地图不完整，不静默删除旧区域。配置在本地 YAML 调整。

默认增加**存图层时序静态过滤**，现已采用放宽组：新体素至少 3 次独立观察、跨度 0.6 s 才确认；
已有点仅在运动/可靠位姿门限内，被有效回波前的自由射线至少 3 次穿过、跨度 0.4 s 才清除。
遮挡或没看到不删图，历史区域不按年龄删除。只改变存图缓存和建图显示，实时 ikd-Tree/
EKF 不变，定位参考 PCD 只读。`static_filter:=false` 关闭作对照；本地 YAML 可调整
`static_map` 段。站定的人仍可能入图，离开后需要重访、真正照到旧位置，不能保证全部剔除。
参数、状态、性能边界与现场验收见 [静态存图过滤](doc/STATIC_MAP_FILTER.md)。

两阶段可独立测试：`map_confirm:=true/false` 控制入图确认，`ray_clear:=true/false`
控制射线清理，均支持 auto 遵循 YAML。常用门限可直接传
`confirm_hits:=3 confirm_seconds:=0.6 confirm_interval:=0.1` 与
`clear_hits:=3 clear_seconds:=0.4 clear_interval:=0.1`；行走门控可传
`clear_max_speed:=1.0 clear_max_angular_speed:=1.0`（m/s、rad/s）。这些现在就是默认值，
普通建图无需重复传参；旧本地 YAML/显式 CLI 仍优先，不会自动改写。
新默认可能增加行人入图/误删，仍需现场验收；更新源码先重新 build，之后调参只需重启。
完整映射和原严格组对照指令见上方文档。

无显示需求时全图发布默认关闭；建图 `--rviz` 自动开启有限显示副本，远程 PC 可传 `publish_map:=true`。默认有订阅者才每 5 s 发布，显示最多 100000 点，定位 reference_map 也用单独显示副本，不改变匹配地图。

持续匹配有质量门控：坏帧不发布可信位姿、不插点、不录制；`tracking → degraded → lost` 见 `/tracking/status`。丢失状态锁定，自动定位先停止录制再 `/relocalize`，手动定位和建图需重启。局部窗口默认 400 m/触发范围 100 m，启动校验窗口边长 > 3×范围。门限是待实机调参初值，不是精度或安全保证。

### 定位

```bash
bash scripts/run.sh localization \
  map_path:=pcd_map/20260916_160000_123456_lab_a.pcd search_radius:=3.0
# 切换到另一个场景时只需换地图路径，不改 YAML：
bash scripts/run.sh localization \
  map_path:=pcd_map/20260917_100000_123456_lab_b.pcd
# 按需开启计算端 RViz2，默认不开：
bash scripts/run.sh localization \
  map_path:=pcd_map/20260916_160000_123456_lab_a.pcd --rviz
# 已 source 的容器内也可直接启动：
ros2 launch fast_lio mid360.launch.py mode:=localization \
  map_path:=/workspace/humble_space/fastlio-mid360_space/pcd_map/场景地图.pcd
```

**定位启动必须显式传入 `map_path`**。不传、文件不存在、后缀不是 `.pcd`、空文件或不可读，launch 会在启动节点/driver 前报错；不会默认加载示例地图或自动选择最新地图。该要求同样适用于旧定位 launch、通用 `mapping.launch.py`，以及使用定位 YAML 的离线 replay。YAML 中不再固定 `localization.map_path`。

上述 PCD 是运行时的参考地图，不是编译依赖。换地图、搜索半径或 RViz2 开关只需
重新启动，不需要重新编译；修改 C++ 源码才需要再次运行 `build.sh`。

脚本会把启动目录固定在工作区根目录，因此可以使用 `pcd_map/xxx.pcd` 相对路径；手动 `ros2 launch` 的相对路径以当前终端 CWD 为准。宿主机快捷脚本最终在 Docker 内运行，绝对路径必须是**容器可访问的路径**，不是 `/home/shawntao/...` 的宿主机路径。

`pcd_map/test.pcd` 只是原工程带来的启动示例，不能假定它对应当前场地，必须显式选择才能加载。定位默认先在建图原点 `[0,0,0]` 周围 3 m 内搜索初始位置和完整 360° 朝向；启动时保持静止，等待 `/localization/status` 为 `ready`。可以通过 `search_radius:=3.0` 覆盖半径，默认沿重力方向限制高度偏差 ±0.5 m；先通过重力对齐处理小倾斜，再搜索 xyz/yaw，不进行任意 6-DOF 搜索。粗搜索、精配准和新扫描复核通过后才允许跟踪，开始录制还须等待 `/tracking/status` 为 `tracking`；失败不回退到地图原点。

初始位姿 `[x,y,z,yaw]`、重叠率、残差可从 `localization.matched_pose` / `matched_overlap` / `matched_rmse` 节点参数读取；失败后可调用 `/relocalize` 重试。完整状态、调参、边界与测试见 [启动重定位说明](doc/STARTUP_RELOCALIZATION.md)。显式 `relocalize:=false` 恢复原来的 YAML `localization.initial_pose` 与 `/initialpose` 直接播种方式；自动模式不接受直接位姿跳变。

定位模式只读地图，发布 latched `/reference_map`，拒绝 `/map_save`。新图附带 `.pcd.json`，自动校验点数/文件校验和、标定/时间及预处理参数，拒绝容量截断地图；正式采集建议加 `map_metadata:=strict`。旧图默认兼容并告警，不宣称元数据已验证。建图和定位的 `preprocess`、标定外参及 IMU 协方差须保持一致；本工作区包含检查这些参数一致性的测试。不同场景必须使用各自的实际地图；重复结构、低质量地图或超出搜索范围不保证重定位成功。

### NX 计算、AGX 独立显示

按本文开头的常用指令，NX 用 `run.sh`，AGX 用 `rviz.sh`；各自在自己的终端执行。
NX 建图传 `publish_map:=true`，地图路径只在 NX 定位启动时输入。

两个 RViz 预设固定 camera_init；定位预设已添加 /reference_map 并使用 Transient Local。
两端 DDS/domain 一致，不使用 ROS_LOCALHOST_ONLY=1。具体环境选择和图形容器
约束见 [脚本说明](scripts/README.md)。原 fastlio.rviz 仍可作为自定义配置选择。

原 GT 记录工具保留，也安装为 ROS 可执行脚本：

```bash
ros2 run fast_lio gt_record.py start --source manual --instr lab_01
ros2 run fast_lio gt_record.py stop
ros2 run fast_lio gt_postprocess.py --help
```

详细 GT 流程见 [doc/GT_SYSTEM_README.md](doc/GT_SYSTEM_README.md)。CSV 默认位于工作区 `records/`，开始录制即创建文件、逐样本追加，默认每 1 s 同步；可用 `record_dir:=/其他目录` 修改。显示轨迹限制点数，不截断 CSV；断流/失匹配造成的时间缺口不可当作连续真值。工作区 bind mount 可持久化记录，但 Git/镜像/部署 ZIP 不含这些数据。

### 无雷达启动与离线重放

```bash
# 只启动 FAST-LIO；无 bag 输入时它等待雷达/IMU数据：
bash scripts/run.sh replay map_name:=lab_a_offline rviz:=false use_sim_time:=true
# 或重放已有地图定位模式：
bash scripts/run.sh replay config_file:=mid360_localization.yaml \
  map_path:=/容器内路径/参考地图.pcd use_sim_time:=true
```

在另一个已 source 的容器终端播放 bag：

```bash
ros2 bag play /容器内路径/bag目录 --clock --topics /livox/lidar /livox/imu
```

只播放原始两个传感器 topic，避免原 bag 中旧 `/Odometry`、`/tf` 或其他结果 topic 与当前解算混在一起。可添加 `--rate 0.5` 降低重放速度。重放时默认不启动 driver、不再次录 bag。

旧 launch 名称 `mapping_mid360.launch.py`、`mapping_mid360_localization.launch.py`、`localization_replay.launch.py` 作为兼容入口保留，委托统一 launch；新流程推荐 `mid360.launch.py` / `scripts/run.sh`。参数列表：

```bash
ros2 launch fast_lio mid360.launch.py --show-args
```

## 7. 测试边界与依赖

`bash scripts/test.sh` 先运行两个包的 `colcon test` / `colcon test-result --verbose`，再在 localhost DDS 域执行无硬件启动测试。测试包括：

- x86-64 / ARM64 的 SDK 路径与 ELF 检查，拒绝不支持的架构及错误 SDK override。
- colcon 包依赖、CustomMsg/topic 设置、建图与定位物理参数一致性、示例地图和第三方源码完整性、launch Python 语法。
- driver 使用专门的 `tests/MID360_loopback.json` 初始化 SDK；不是实机配置，不修改实际 JSON。
- FAST-LIO 建图与定位节点启动、传感器订阅类型、ROS 服务注册、参考 PCD 发布、定位模式下拒绝存图、空建图结果返回失败。
- 定位必填 `map_path`、文件/场景名校验、真实 PCD 写入/读回、同会话快照更新与跨会话覆盖保护。
- Bash / Zsh 语法与 DDS 环境、容器自动选择/歧义拒绝、含空格路径、Docker 入口和 ZIP 输出保护；这些工具测试使用本地假 Docker，不连接 daemon。
- 真实 Git 忽略/属性匹配（临时元数据，不初始化项目）、本地配置独占创建、IP 联动/校验、默认配置不变、相对 JSON/YAML 启动与非法 JSON 拒绝。
- 有界搜索/启动门控、重力对齐、持续跟踪状态、窗口校验、体素容量和快照、CSV 持久化与异常尾行恢复、元数据兼容和完整性拒绝。
- 存图时序确认、自由空间重复证据、遮挡/端点保护、历史区域保留、候选/帧输入/射线有界、关闭过滤兼容旧机制；合成策略测试不代替实机动态场景验收。

测试不录 bag、不写参考地图、不发送真实雷达指令；会生成测试日志，运行调试日志默认关闭。启动自检不能代替实际雷达收包、IMU 初始化、连续点云配准、建图精度或 ARM64 原生编译验证。日志见 `log/smoke/`，构建日志见 `log/latest_build/`。最新改动的实际验证边界见 [VERIFICATION.md](VERIFICATION.md)。

## 8. 部署打包

```bash
bash scripts/package.sh
# 可指定工作区外的新 ZIP 路径；不会覆盖已有包：
bash scripts/package.sh /其他目录/fastlio-mid360_jetson.zip
```

包含源码、两套 SDK、正式 DDS/Docker/doc/scripts/tests 和 `pcd_map` 地图，
排除 build/install/log/bags/records、scripts/backup、Git/Python 缓存、旧 maps 与 reference、本地配置和私有环境文件。
自动检测 ZIP CRC、第三方源码、SDK ELF、脚本权限与不安全路径，生成同名
`.zip.sha256`，拷到 Jetson 后先 `sha256sum -c` 再用 Linux `unzip` 解压。
ZIP 不包含 Docker 镜像或 apt 离线依赖，首次准备环境仍需网络。

前次验证容器具备编译/启动所需依赖，但当前 WSL 主机没有 ROS、Docker daemon 不可用，不能据此宣称最新节点代码已编译通过。换环境可先初始化/更新 rosdep，再在 Humble 容器中执行 `rosdep check --from-paths src --ignore-src --rosdistro humble`；确实缺依赖时才执行 `rosdep install --from-paths src --ignore-src --rosdistro humble -y`。核心依赖包含 ament/rosidl、rclcpp/components、pcl_ros/pcl_conversions、tf2_ros、std_srvs、Eigen/PCL、APR、Python3 development、OpenMP 及 colcon。

常见排查：

- `Unsupported Livox SDK architecture`：现有库仅支持 64 位 x86/ARM，不应强制使用错误指令集。
- `SDK architecture mismatch`：清空错误 `LIVOX_SDK_ROOT` override；换架构后重新构建，不复用旧产物。
- 节点启动但无点云：检查实际 JSON、容器网卡地址、雷达电源和 UDP 网络；driver 的点云 publisher 可能直到有数据才建立。
- `failed to load reference map`：检查容器内 PCD 路径；`map_path` 与 `map_output` 可用绝对路径，工作区脚本会固定启动 CWD。
- 无法打开 RViz：先 `rviz:=false` 验证计算链路，再检查 DISPLAY / X11 / WSLg；脚本不会自行修改 X server 权限。
- 编译被 `Killed`：降低 `FASTLIO_BUILD_JOBS` / `MAKEFLAGS`，检查容器内存。
- `Clock skew detected`：本环境的 bind mount 曾出现亚秒级时间戳偏差。不要改源码来绕过；检查宿主机/WSL 与 Docker 的时间同步，再增量构建并运行自检。本次最终二进制还核验了退出处理符号与运行行为。

## 9. Git 版本管理

`.gitignore` 按构建、数据、本地配置和工具缓存分类；SDK / 默认 JSON/YAML / 包清单
必须跟踪，场景 PCD 默认忽略（保留 test.pcd）。`.gitattributes` 统一文本 LF，
SDK / 地图等不做文本转换，没有强制 Git LFS。详细流程见
[doc/VERSION_CONTROL.md](doc/VERSION_CONTROL.md)；项目不会被脚本自动 git init / commit。
