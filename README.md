# FAST-LIO + Livox MID360：ROS 2 Humble 统一工作区

原电脑目录：`/home/shawntao/workspace/humble_space/fastlio-mid360_space`。原验证 Docker 容器：`3018b9a5759e`；挂载目录：`/workspace/humble_space/fastlio-mid360_space`。部署后可以放在任意路径；脚本不固定原容器 ID，按工作区 bind mount 查找。

Jetson 首次部署请读 [doc/JETSON_DEPLOY.md](doc/JETSON_DEPLOY.md)：源码 ZIP 需在 ARM64 上重新编译，不携带原电脑的编译产物。Docker 镜像环境与 [DDS 配置](dds_config/README.md) 也已更新。

driver 与 FAST-LIO 已整理为同一 `src/` 下的两个 ament 包。只需 source ROS Humble，再执行一次 `colcon build --symlink-install`，colcon 会依据 `fast_lio/package.xml` 的依赖关系先编译 driver、再编译 FAST-LIO，不需要中途 source driver，也不需要预先安装 SDK 到 `/usr/local`。

## 1. 快速使用

在工作区根目录执行：本机已有 Humble 时默认本机运行；否则自动解析 bind mount 进入匹配的运行容器。显式设置 `FASTLIO_CONTAINER` 或 `FASTLIO_NATIVE=0` 可以锁定 Docker。

```bash
cd /home/shawntao/workspace/humble_space/fastlio-mid360_space
bash scripts/build.sh
bash scripts/test.sh                 # 无雷达：配置、架构、节点启动自检
bash scripts/check_network.sh        # 实机运行前检查；只读，不修改网卡
bash scripts/run.sh mapping map_name:=lab_a rviz:=true record_bag:=true
```

在**容器内**手动编译、启动：

```bash
cd /workspace/humble_space/fastlio-mid360_space
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source scripts/setenv.bash
ros2 launch fast_lio mid360.launch.py mode:=mapping map_name:=lab_a rviz:=false
```

`local_setup.bash` 在已 source Humble 后加载本工作区；也可直接 `source install/setup.bash`。使用 zsh 时相应换成 `setup.zsh` / `local_setup.zsh`。不要混入旧 driver / FAST-LIO 工作区的 overlay，尤其不要沿用旧 `scripts/x86_setenv.sh` 中的失效路径与固定网卡 DDS 配置。

交互 ROS CLI 推荐 `source scripts/setenv.bash`（Zsh 用 `setenv.zsh`），它还配置与运行脚本一致的默认 DDS 域 18；仅 source ROS/install 不会设置 domain。PC、Jetson 和另开终端必须保持同一个 `ROS_DOMAIN_ID`。

低内存机器建议 `bash scripts/build.sh`：默认两个编译任务、包级串行，不删除任何已有产物。手动编译可用：

```bash
MAKEFLAGS='-j2 -l2' colcon build --symlink-install
```

脚本配置：

```bash
FASTLIO_CONTAINER=<容器名或ID> bash scripts/build.sh
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

```text
fastlio-mid360_space/
├── src/
│   ├── livox_ros_driver2/    # ROS2 Humble driver、消息、配置、第三方头文件
│   └── FAST_LIO/            # fast_lio 包，含完整 ikd-Tree / IKFoM 源码与 GT 工具
├── livox-sdk-x86/           # x86-64 预编译 SDK：include/ + lib/
├── livox-sdk-arm/           # ARM64 / aarch64 预编译 SDK：include/ + lib/
├── pcd_map/                 # 与 src/ 同级的统一地图目录
│   └── test.pcd             # 显式选择的自检示例，不作为默认定位地图
├── scripts/                # 编译、运行、自检、Docker 调度及网络检查
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

原 `fastlio-space` 未修改。原 `livox_space` 的 driver 只修改了 CMake 的默认 ROS2/Humble 配置与 SDK 查找，并新增 `cmake/livox_sdk.cmake`；旧源码、SDK 和编译产物均保留。新工作区的完整配置与原工程独立，不引用原工程的源码或 SDK。`humble_space` 根目录下的 DDS、Docker、文档同步正式版本；旧 scripts 名称仅委托新工作区。

新副本还修正了 FAST-LIO 的退出处理：不再覆盖 rclcpp 自带的异步 SIGINT handler，避免在原 POSIX signal handler 内调用 shutdown() 导致 Ctrl+C 退出卡住。配准、建图与定位算法不变。

地图管理已统一：有效建图帧只累积一次，不再依赖地图发布定时器来拼接存图数据；`/map_save` 与正常 Ctrl+C 存储同一份地图。旧 `maps/` 内的示例保留用于兼容，不再是建图输出目录。

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
# 无需 ROS / Docker / sudo；示例 IP 请换成目标机器真实地址
bash scripts/init_local_config.sh --name jetson \
  --host-ip 192.168.123.18 --lidar-ip 192.168.123.114
bash scripts/check_network.sh config/local/MID360.jetson.local.json
bash scripts/run.sh mapping lidar_config:=config/local/MID360.jetson.local.json map_name:=lab_a
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

### 建图

```bash
bash scripts/run.sh mapping map_name:=lab_a rviz:=true record_bag:=true
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
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

地图统一保存在与 `src/` 同级的 **`pcd_map/`** 下，默认命名为：

```text
pcd_map/20260916_160000_123456_lab_a.pcd
        启动日期_启动时间_微秒_自定义场景名.pcd
```

`map_name` 默认 `map`，可以使用中文、字母、数字、下划线和连字符，不含 `.pcd` 后缀或路径分隔符。时间戳在节点启动时生成，使用所在容器/系统的时区。同次运行中，`/map_save` 会更新当前会话文件，正常 Ctrl+C 则把最终地图保存到同一文件；不同运行默认使用不同时间戳，不覆盖之前的场景地图。启动日志会打印完整的 `Mapping session output` 路径。

`map_dir:=/其他目录` 可以改输出目录；保留高级兼容项 `map_output:=/完整路径/new_map.pcd`，显式指定完整输出路径时不再生成时间戳，且启动会拒绝已经存在的目标文件。保存先写同目录临时文件，再原子替换当前会话地图；空地图、写入失败均返回失败，不会报告成功。

无有效建图数据时，`/map_save` 返回 `No mapping points available`，Ctrl+C 不创建空 PCD；不应预期出现有效地图或里程计数据。无论 `publish.map_en` 或扫描显示开关是否开启，只要 `pcd_save.pcd_save_en=true` 就会累积建图结果。

`record_bag` 默认 false，主动开启后保存到工作区 `bags/<mode>_<时间戳>/`；可用 `bag_dir:=/其他路径` 和 `extra_bag_topics:='/topic_a /topic_b'` 配置。此工作区每个会话保存一份完整地图，不使用 `pcd_save.interval` 分片；逐帧累积会随时间增长，应监测长时间建图的内存。配准算法、标定与操作员过滤逻辑没有改动。

### 定位

```bash
bash scripts/run.sh localization \
  map_path:=pcd_map/20260916_160000_123456_lab_a.pcd rviz:=true
# 切换到另一个场景时只需换地图路径，不改 YAML：
bash scripts/run.sh localization \
  map_path:=pcd_map/20260917_100000_123456_lab_b.pcd rviz:=true
# 已 source 的容器内也可直接启动：
ros2 launch fast_lio mid360.launch.py mode:=localization \
  map_path:=/workspace/humble_space/fastlio-mid360_space/pcd_map/场景地图.pcd
```

**定位启动必须显式传入 `map_path`**。不传、文件不存在、后缀不是 `.pcd`、空文件或不可读，launch 会在启动节点/driver 前报错；不会默认加载示例地图或自动选择最新地图。该要求同样适用于旧定位 launch、通用 `mapping.launch.py`，以及使用定位 YAML 的离线 replay。YAML 中不再固定 `localization.map_path`。

脚本会把启动目录固定在工作区根目录，因此可以使用 `pcd_map/xxx.pcd` 相对路径；手动 `ros2 launch` 的相对路径以当前终端 CWD 为准。宿主机快捷脚本最终在 Docker 内运行，绝对路径必须是**容器可访问的路径**，不是 `/home/shawntao/...` 的宿主机路径。

`pcd_map/test.pcd` 只是原工程带来的启动示例，不能假定它对应当前场地，必须显式选择才能加载。请使用当前场地建出的地图，并在 `mid360_localization.yaml` 中设置正确的 `localization.initial_pose`。地图选择不等于自动重定位；不同场景还需要匹配的初始位姿。定位模式只读地图，发布 latched `/reference_map`，拒绝 `/map_save`。建图和定位的 `preprocess`、标定外参及 IMU 协方差须保持一致；本工作区包含检查这些参数一致性的测试。

原 GT 记录工具保留，也安装为 ROS 可执行脚本：

```bash
ros2 run fast_lio gt_record.py start --source manual --instr lab_01
ros2 run fast_lio gt_record.py stop
ros2 run fast_lio gt_postprocess.py --help
```

详细 GT 流程见 [doc/GT_SYSTEM_README.md](doc/GT_SYSTEM_README.md)。CSV 默认位于运行环境用户家目录 `~/Record_Path`，Docker 中不在工作区挂载内，删除容器前须单独拷出。

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

测试不录 bag、不写参考地图、不发送真实雷达指令；会生成测试日志和原算法的启动 debug 日志。启动自检不能代替实际雷达收包、IMU 初始化、连续点云配准、建图精度或 ARM64 原生编译验证。日志见 `log/smoke/`，构建日志见 `log/latest_build/`。

## 8. 部署打包

```bash
bash scripts/package.sh
# 可指定工作区外的新 ZIP 路径；不会覆盖已有包：
bash scripts/package.sh /其他目录/fastlio-mid360_jetson.zip
```

包含源码、两套 SDK、正式 DDS/Docker/doc/scripts/tests 和 `pcd_map` 地图，
排除 build/install/log/bags、Git/Python 缓存、旧 maps 与 reference、本地配置和私有环境文件。
自动检测 ZIP CRC、第三方源码、SDK ELF、脚本权限与不安全路径，生成同名
`.zip.sha256`，拷到 Jetson 后先 `sha256sum -c` 再用 Linux `unzip` 解压。
ZIP 不包含 Docker 镜像或 apt 离线依赖，首次准备环境仍需网络。

当前容器已经具备编译/启动所需依赖，不需要额外下载；其 rosdep 数据库尚未初始化，不能通过 rosdep check 审计。换环境可先初始化/更新 rosdep，再在 Humble 容器中执行 `rosdep check --from-paths src --ignore-src --rosdistro humble`；确实缺依赖时才执行 `rosdep install --from-paths src --ignore-src --rosdistro humble -y`。核心依赖包含 ament/rosidl、rclcpp/components、pcl_ros/pcl_conversions、tf2_ros、std_srvs、Eigen/PCL、APR、Python3 development、OpenMP 及 colcon。

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
