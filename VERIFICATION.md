# 验证记录

日常指令在 [主 README 开头](README.md)、[脚本说明开头](scripts/README.md) 和
[Jetson 指南开头](doc/JETSON_DEPLOY.md)；本文件只记录验证范围，不替代启动指南。

## 默认采用放宽存图配置（2026-09-18，最新源码）

- 按用户要求，把前一版仅作为测试例子的放宽组改为默认：入图 3 次、跨度 0.6 s，
  清理 3 次、跨度 0.4 s，两者间隔 0.1 s；清理速度/转速上限 1.0 m/s、1.0 rad/s。
  同步默认 YAML、节点声明、launch 缺省校验和独立 C++ 策略，普通建图无需传这八项。
- 总开关和两阶段仍默认开启。端点/遮挡保护、射线范围/预算、位置标准差 0.10 m、
  帧间隔 0.5 s、容量及可信帧门控不变，实时树/EKF、物理参数、定位配置不变。
  旧本地 YAML/显式 CLI 不自动覆盖；缺少清理间隔仍继承其入图间隔，原严格组可显式恢复。
- 新默认允许更多行走帧清理，也降低确认/删除证据门槛；可能增加动态点入图或误删。
  不保证四足实机误删率、清理时限或性能。文档同步新默认并保留原严格组对照指令。
- 主机验证：63 项 Python 回归通过；新增 YAML/节点/launch 默认一致性和旧严格本地
  配置不被静默放宽检查。GCC 13 / C++17 / -O2 / -Wall -Wextra -Werror 静态地图策略
  测试通过，新增默认入图/清理时序及运动许可验证；ASan/UBSan 通过，LeakSanitizer 关闭。
- 文档 97 个 Bash 代码块/脚本引用、三份主要速查区一致性、Python/shell 语法和
  git diff --check 通过。默认雷达 JSON 和 SDK 不修改，本地实机 IP 配置不覆盖。
- 本机仍无 Humble/Eigen/PCL，Docker WSL integration 不可用；完整 ROS 节点编译、
  ROS smoke、ARM64 和机器人性能/精度仍待 Jetson 验收。更新源码后先重新 build/test，
  使用旧自定义 YAML 的用户须自行更新其 static_map 段或切回默认配置。

## 两层存图过滤独立调参（2026-09-18，默认放宽前）

- 增加独立的入图确认/射线清理开关及启动参数；保留原默认门限，未自动放宽。
  确认、清理分别配置次数、跨度和观察间隔；旧 YAML 缺少清理间隔时仍继承原间隔。
  支持单次确认/清理、零跨度/间隔，同帧重复点和多条射线仍只计一次证据。
- 清理运动门限新增可配置帧间隔；速度、位姿/IMU 峰值转速、位置标准差分别可覆盖。
  启动前检查非法数值及有效阶段的组合约束；总开关优先，定位不接受建图过滤调参。
  配置只影响存图层，实时 EKF/ikd-Tree、参考地图及 TF 初始化/参数保护顺序不变。
- /tracking/status 增加实际生效阶段/间隔与 archive_clear_gate；地图 JSON 记录新增参数。
  同帧端点保护、端点截断禁止清理和容量不完整锁存均保留。文档给出四种开关组合、
  放宽后的测试例子及误删/行人残留权衡，不把例子当成实机最佳组合。
- 61 项主机 Python 回归通过，包含明确标注的 launch action stubs，不替代真实 ROS；
  C++ 静态地图策略覆盖四种组合、独立间隔、零间隔单帧计票、运动门限放宽及旧配置。
  GCC 13 / C++17 / -Wall -Wextra -Werror 原生测试和 ASan/UBSan 检查通过；
  LeakSanitizer 仍关闭，未做完整节点内存检查。原运行/重定位策略原生回归也通过。
- 8 项 CMake SDK 架构选择/拒绝、97 个文档 Bash 代码块与引用路径、三份速查区一致性、
  Python 语法、git diff --check 通过。默认雷达 JSON SHA256 未变，SDK 未修改。
- 完整 CTest 仍登记 17 项；真实 smoke 已扩展两阶段状态和在线修改拒绝检查。
  本机无 Humble/Eigen/PCL，Docker WSL integration 不可用，未完成本次完整节点编译、
  真实 ROS smoke、ARM64 或机器人建图精度/性能验收。Jetson 更新源码后先 build/test，
  以后只改启动参数或本地 YAML 则正常退出后重启，无需再次编译。

## 存图层时序静态过滤（2026-09-17）

- 新增 ROS/PCL 无关的 static_map.hpp：候选多次/时间跨度确认、同帧去重、有限
  自由射线反证、近回波/端点/邻域保护、负证据清零与过期、候选 TTL/LRU 有界回收。
  已确认历史区域不按年龄删除；站定行人仍可能确认，清理取决于真实可见性。
- 仅接入独立存图缓存/显示/PCD，不修改 EKF 和实时 ikd-Tree，不写定位参考 PCD。
  增加低速/位姿协方差/相邻位姿转速及扫描 IMU 峰值转速门控；坐标变换缓存也有界。
  候选/确认/帧输入截断锁存 complete=false；端点被截断的帧禁止任何自由空间清理。
- 默认建图启用，static_filter:=false 保留原体素/原始追加对照；JSON 加入全部门限、
  策略和统计，/tracking/status 加入 static_map。schema=1 旧图兼容规则不变。
  TF 初始化/启动参数保护顺序不变；过滤参数运行时修改仍被拒绝。
- GCC 13 / C++17 / -O2 / -Wall -Wextra -Werror 原生新策略测试通过：
  走动/短暂停留、站定后离开、遮挡/端点保护、正证据重置、负证据过期、历史保留、
  帧重复/缓存及射线上限、调用方省略端点、无效输入/时间、关闭兼容、重新确认，
  以及固定随机种子的 400 组通用 3-D/负方向射线遍历。原运行策略、启动重定位策略
  原生回归也通过。新策略 ASan/UBSan 检查通过，关闭 LeakSanitizer（当前执行环境限制）。
- 合成主机基准：先积累 500000 个全局点，再输入 20000 点 × 300 帧；测得中位
  12.41 ms、P95 21.98 ms、最大 31.17 ms，进程峰值 RSS 68568 KiB。
  测试使用简化 double Point，不含 ROS/PCL/EKF、坐标变换、显示或保存，不是 Orin NX
  性能/整机内存保证，真实场景和共享主机负载也会改变结果。
  从工作区根目录以 C++17 / -O2 编译 static_map_test.cpp 后，执行该测试程序的
  `--benchmark-large` 选项并用 `/usr/bin/time -v` 观察独立进程峰值内存即可复现。
- 55 项主机 Python 回归、8 项 CMake SDK 架构选择/拒绝、96 个文档 Bash 代码块/
  脚本路径、三份速查区一致性、Python 语法和 git diff --check 通过。
  全量 unittest 尝试的其余 10 项真实 launch 测试因缺少 ROS launch 模块未能执行，
  不把 action stubs 当成真实 ROS。默认雷达 JSON SHA256 与改动前一致，SDK 未修改。
- CTest 增加 archive_static_map_policy，完整 ROS 环境现在登记 17 项；真实 smoke
  增加两模式过滤状态与运行时开关拒绝检查，但本机无 Humble/Eigen/PCL，Docker WSL
  integration 不可用，**未完成最新 Humble/ARM64 节点编译或真实 smoke**。
  Jetson 须重新 build/test，再按 [动态场景验收](doc/STATIC_MAP_FILTER.md) 对照实测。

部署包是待 Jetson 编译验收的源码包；不携带本机配置、轨迹/录包、旧 build/install/log。
文档已同步默认过滤、对照开关、性能/误删边界，未新增第三方动态过滤依赖。

## TF QoS 初始化与参数保护顺序修复（2026-09-17）

- 针对实机日志中 `qos_overrides./tf.publisher.durability` 被 startup-only 回调
  拒绝并导致 FAST-LIO abort 的问题，将参数保护回调注册移至构造函数末尾，
  位于 TF、发布器、订阅器、定时器和服务创建之后、Node init finished 日志之前。
- 参数保护回调内容与修复前逐字一致；不放开运行时 QoS、物理参数或跟踪门限修改，
  不修改雷达 IP JSON、SDK、建图/定位配置或算法。
- 新增主机源码顺序回归，确认修复前源码会失败、修复后通过。
  全部 53 项主机 Python 回归通过，扩展 smoke 的 Python 语法及 git diff --check 通过。
- 真实 ROS 无硬件 smoke 已增加建图/定位两种模式的 TF 发布器和只读 QoS 参数检查，
  验证在线修改启动参数、TF QoS 和非法采样步长被拒绝且值不变；验证录制标签/
  合法采样步长仍可修改，并恢复测试前值。仅操作测试创建的隔离节点，不写参考地图。
- 本主机仍无 ROS Humble，Docker WSL integration 不可用，未执行真实 ROS 编译
  或上述扩展 smoke。同步源码后须在 Jetson/可用容器中重新 build，再执行 scripts/test.sh；
  旧 install 和历史 ZIP 不包含此次修复，不能仅重启旧二进制。

## 统一交互环境入口（2026-09-17）

- 新增 `source scripts/env.sh`，自动选择 Bash/Zsh 的现有 setenv 实现，加载
  Humble、可选工作区 overlay 与启动脚本一致的 DDS；显示实际工作区、domain、RMW。
  顶层现为 11 个入口，原 setenv.bash/zsh 继续保留供现行脚本及兼容调用使用。
- 必须 source，显式拒绝 `bash/zsh scripts/env.sh`；不修改 shell 启动文件、
  网卡或其他终端。ROS 在 Docker 内时提示先进入对应容器，不能向宿主机注入环境。
- 未编译支持 ROS/DDS-only；未安装 Humble、底层 setup 失败或 DDS 选择无效时
  返回错误，不打印加载成功。保留显式 DDS/domain/localhost 设置，localhost-only=1 告警。
- 新增 9 项主机环境入口回归：真实 Bash/Zsh 执行、路径含空格、匹配 shell、
  默认 DDS/覆盖、ROS-only、缺失/失败、错误 DDS、localhost 告警及旧入口错 shell 拒绝。
  使用明确的假 ROS/overlay setup 文件，只验证 shell 行为和环境变量，不代表 ROS 编译。
- 全部 52 项主机 Python 回归通过，90 个文档 Bash 代码块与路径检查通过，
  三份主要速查区一致，git diff --check 通过。当前主机无 Humble，实机
  `ros2 topic list`、NX↔AGX 发现及 RViz 显示仍需目标机验证。
- 部署归档检查将 env.sh 列为必需文件；新版 ZIP 包含此入口，不覆盖历史包。

## 文档速查前置与同步（2026-09-17）

- 三份主要文档开头使用完全一致的 NX 编译/自检、NX 建图、AGX 建图显示、
  NX 定位、AGX 定位显示命令；所有实机例子显式选择本地雷达 JSON。
- 启动重定位、长期运行、GT、Git、配置、地图、DDS、Docker 共 11 份使用文档
  首个 Bash 命令均在前 15 行内；历史背景、目录结构和高级选项留在后文。
- 分开计算端/查看端及并行终端，说明建图远程显示须 publish_map:=true、
  编译不需要 PCD、RViz/rosbag 默认关闭，以及 PCD/JSON 保存配对。
- 只读文档检查通过：86 个 Bash 代码块语法正确、引用的 scripts/docker 脚本存在，
  现行示例不调用已归档的旧顶层入口。三份主要速查区逐字一致。
- 重新执行 43 项主机 Python 回归，全部通过；git diff --check 通过。
  本次仅整理文档，不新增真实 Humble/ARM64 编译、Qt GUI 或跨机通信验证。

## 脚本精简和独立 RViz 入口（2026-09-17）

- 顶层由 18 个脚本精简为 10 个入口；5 个历史名字移到 scripts/backup/，保留可用
  wrapper，必需 helper 放 scripts/lib/。没有删除历史脚本，现行入口不依赖 backup/。
- 计算端统一 run.sh mapping/localization/replay；rviz.sh mapping/localization 只开
  查看器，自动加载相同 DDS 默认值，不要求本工作区编译/SDK/地图。
- 增加 mapping.rviz / localization.rviz 预设，定位 /reference_map 使用 Transient Local；
  统一 launch 的 --rviz 也按模式选预设。旧 fastlio.rviz 可作为自定义配置保留。
- 更新文档、移动后路径、Docker 调度白名单和 ZIP 必需文件/排除规则；部署 ZIP 和
  Docker 上下文不含 backup/，但仍含全部 lib/ 实现和两份 RViz 预设。
- 主机 43 项 Python 检查通过（17 工具/布局/查看器、8 参数、9 Git/本地配置、
  3 运行配置/CSV、6 无 ROS 工作区契约）。查看器执行测试使用明确的假 ROS 环境及
  假 rviz2，仅验证命令、参数和 DDS，不代表 Qt GUI 渲染。

仍无可用 ROS/Docker，最新真实 Humble action/节点启停、RViz 图形加载及
NX↔AGX 跨机发现、参考图晚订阅和点云实时显示待现场验收。
源码归档测试省略历史 wrapper 专用用例（backup/ 不随部署包发出），其余工具
用例不依赖备份目录。

## 长期运行、存图和数据可信度更新（2026-09-17，当前代码）

本节优先于后面的历史记录。当前 WSL 主机仍没有 `/opt/ros`、Eigen/PCL 开发环境；
`docker ps` 仍提示 WSL integration 不可用。本次**没有完成最新 Humble 节点编译、
ROS 无硬件 smoke、ARM64 原生编译或真实 MID360 精度/实时性能验收**。
2026-09-16 的成功构建不覆盖本次源码。

代码增加：

- 局部窗口 400 m / det_range 100 m 及启动双层参数校验；可信更新后才裁剪/插点。
- 持续跟踪质量、协方差和墙钟新鲜度门控；队列上限/时间戳回退清理；lost 锁定，
  不输出可信位姿/TF、不录制、不插点。自动定位可显式重定位，手动模式须重启。
- 独立全局 0.1 m 实测代表点体素缓存，默认 2000000 点上限；达到上限拒绝新体素，
  不静默删除旧场地；PCD 标记不完整并在定位时拒绝。
- 后台 PCD/JSON 保存、60 s 检查点、退出最终保存，CRC32/点数/文件大小及物理参数
  校验；保存响应表示入队，须等待 /map_save/status saved。CRC 不是安全签名。
- 参考地图和 IMU 扫描重力对齐后 xyz/yaw 搜索，转换回原图坐标，限制初始倾斜。
- 默认关闭全图显示，按订阅者/间隔/显示点数限制；CSV 默认 records/ 流式同步写，
  保留质量列和中断元数据，显示轨迹有界；读取器只容忍损坏的最终行。
- 当前帧 Odometry 协方差发布顺序、位置/旋转索引与固定轴变换修正。

本次可复现的主机检查：

- 37 项 Python unittest 通过：11 工具、8 启动/元数据 action-stub 契约、9 Git/本地配置、
  3 运行配置/CSV 恢复、6 不依赖 ROS 的工作区契约。stubs 不替代真实 launch 执行。
- 8 项独立 CMake SDK 架构选择/拒绝检查通过；默认雷达 JSON 与 SDK 未修改。
- 两组独立 C++ 策略测试使用 GCC 13、C++17、-O2、-Wall -Wextra -Werror 编译通过：
  体素去重/容量/快照、窗口/质量/恢复/lost、重力旋转、CSV 同步和 CRC；有界搜索、
  重力对齐重建、歧义/失败/超时、初始化/复核/取消/显式重试。
- AddressSanitizer / UndefinedBehaviorSanitizer 策略检查通过；关闭 LeakSanitizer
  (`ASAN_OPTIONS=detect_leaks=0`)，不宣称覆盖泄漏检查或完整 ROS/PCL 节点。
- shell 语法、Python launch 语法和 git diff --check 通过。

仍需在可用 Humble 环境运行 scripts/build.sh / scripts/test.sh；完整 CTest 现在登记
16 项（8 SDK + 8 FAST-LIO 测试入口），本次未执行该完整 ROS 测试流程。
PCL 实际 PCD/JSON 同步写入回归和依赖 numpy 的完整 GT 后处理本次未运行，不能
把策略/CSV 读取测试当作端到端磁盘或轨迹评估验证。
Jetson 请按 [运行安全与验收](doc/RUNTIME_SAFETY.md) 测静止 30 min、闭合误差、多个
起点、断流、保存期间延迟/峰值内存和异常退出恢复。没有新增闭环或全局位姿图优化。

新的部署 ZIP 是**待目标机编译验收的源码测试包**；旧 ZIP 不覆盖，轨迹/录包和本地
配置不随包迁移，地图和配套 JSON 必须配对保管。

## 文档和 RViz2 脚本选项更新（2026-09-17）

- README、启动重定位、Jetson、GT 文档分清编译与运行：编译不需要 PCD，
  `map_path` / `search_radius` 是定位启动参数，修改地图和运行开关无需重新编译。
- 统一启动脚本和兼容入口支持 `--rviz` / `--no-rviz`，默认关闭，保留
  `rviz:=true/false`；多次指定只转发最后一次选择。`--help` 不依赖 ROS/Docker/地图。
- 主机 11 项工具测试和 5 项参数流转测试通过，覆盖三种模式、兼容脚本、
  带空格地图路径、帮助、RViz2 默认不创建/开启时只创建一个 action，以及开关转换。
  参数测试仍为明确标注的 ROS action stubs，不代表 GUI 渲染验证。
- 真正的 Humble action 配置测试也增加三种模式的 RViz2 默认关闭/显式开启检查，
  但完整 ROS 编译、该项 ROS 测试及 GUI 仍待可用 ROS/Docker 环境运行。

## 启动重定位更新（2026-09-17）

本次主机是 WSL Ubuntu 24.04，GCC 13.3.0 / Python 3.12.3，没有 `/opt/ros`。
Linux Docker 命令提示 WSL integration 未启用；只读检查 Windows docker.exe
也无法连接 Docker Desktop daemon。因此下面的旧 Humble 验证记录仅代表旧快照，
不能视为新启动重定位代码已经完成 ROS 编译或实机验证。

本次已完成：

- 实现独立于 ROS/PCL 的有界四自由度粗搜索、裁剪 ICP、质量/歧义拒绝，
  接入静止积累、后台搜索、新扫描复核、ready/failed 状态和显式重试。
- `g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread` 编译原生回归测试通过。
  合成不对称房间恢复约 2.6 m 位移和 137.5° 朝向，覆盖 ±π 朝向边界、
  独立扫描评估、位置/高度越界、错误场景、重复地点歧义、取消、超时和参数拒绝。
  耗时与误差仅为合成测试，不代表 Jetson 实测性能或精度。
- 原生启动状态机测试覆盖 IMU 门控、运动清除积累、后台 worker 的 busy 拒绝、
  新扫描确认门控、一次性 ready、重试、运动期间取消以及不可信确认扫描拒绝。
- AddressSanitizer / UndefinedBehaviorSanitizer 的检查在主机执行；原始 LeakSanitizer
  不支持当前 ptrace 执行环境，沙箱外检查申请自动审批超时，没有执行。
  后续在沙箱内关闭泄漏检查，内存越界/未定义行为测试通过；不覆盖泄漏检查。
- 4 项主机参数测试通过，使用明确标注的 ROS action stubs，覆盖地图选择、
  半径/手动覆盖、非法半径/模式，以及定位 replay 参数流转；不是 ROS 执行验证。
- 9 项工具测试、9 项 Git/本地配置测试，以及 5 项无 ROS 工程契约检查通过。
  保持建图/定位物理参数一致；默认 LiDAR JSON 与 SDK 未修改。
- ROS 无硬件 smoke 已扩展为检查 waiting_for_imu、无 Odometry 数据和拒绝轨迹录制，
  但受上述环境限制，本次尚未运行。CTest 增加原生搜索/状态机和启动参数测试。
- 临时部署 ZIP 校验通过：279 条目，CRC、必需源码、新重定位文件、两套 SDK ELF
  与脚本权限正确，没有携带本地配置。该包用于主机校验，不覆盖前次交付 ZIP。

待完成：在 Humble 中执行 `scripts/build.sh` / `scripts/test.sh`，随后在 Jetson
使用实际场地地图、多个 ≤3 m 起点和不同朝向，验证匹配率、误匹配、时延、
质量门限、IMU/滤波器位姿重置及跟踪连续性。默认四自由度要求同楼层、姿态直立，
没有任意 roll/pitch 或全图全局定位能力。

本次未覆盖已有 ZIP；需在 ROS 编译/实机验收后重新运行 `scripts/package.sh`。

## 前次 Humble 快照（2026-09-16，未包含启动重定位）

环境：容器 `3018b9a5759e`，镜像 `fastlio-humble:jammy`，ROS 2 Humble，x86-64，GCC 11.4.0，CMake 3.22.1。宿主机工作区 bind mount 到 `/workspace/humble_space`；网络模式为 host。

已完成：

- 从新复制的源码工作区，单次 `colcon build --symlink-install` 完成 `livox_ros_driver2` 与 `fast_lio`，无需中途 source driver 或传入 ROS_EDITION / DISTRO_ROS。
- 最终代码通过 Release 构建，编译参数含 `-O3`；FAST-LIO 使用 C++17。编译脚本的 MAKEFLAGS 限制生效。
- SDK 二进制与原目录逐文件 checksum 一致；新工程 CMake 不依赖旧 SDK / FAST-LIO 的绝对路径。
- driver 实际动态加载 `install/livox_ros_driver2/lib/liblivox_lidar_sdk_shared.so`；该 SDK 指向新工作区的 x86 SDK，不要求 `/usr/local` 安装。
- `CustomMsg` 和 `Pose6D` 的 ROS 接口与 Python 导入成功；FAST-LIO 消息依赖来自本工作区。
- `colcon test-result`：**12 tests, 0 errors, 0 failures, 0 skipped**。包括 8 个 SDK 选择/拒绝测试、1 个原生 C++ 地图存储测试、1 个含 12 项 unittest 的工程配置测试、1 个含 9 项 unittest 的迁移工具测试，以及 1 个含 9 项 unittest 的 Git/本地配置测试。
- 无硬件启动自检：driver 回环地址 SDK 初始化、FAST-LIO 建图/定位图谱与订阅类型、参考 PCD 加载（体素滤波后 15641 点）、定位模式拒绝 `/map_save`。
- 修正自定义 SIGINT 后，建图/定位均有 `process has finished cleanly` 与 `Rebuild thread terminated normally` 日志；没有遗留运行中的自检节点。
- 宿主机 build/test/run 参数显示/网络检查脚本能正确调度到指定容器。

多场景地图管理回归：

- 修改后的两个包已在同一工作区重新完成 Release 编译；新增地图目录 `pcd_map/` 与 `src/` 同级。
- 建图启动日志生成 `pcd_map/20260916_081704_960368_smoke_test.pcd` 会话路径；无雷达输入时 `/map_save` 返回失败，正常 Ctrl+C 后没有生成空地图。
- 原生 C++ 测试完成实际 PCD 写入/读回、同会话快照更新、场景名和时间戳检查、已有目标拒绝，以及写入失败时临时文件清理。
- 定位启动显式加载 `pcd_map/test.pcd`，体素滤波后 15641 点；定位存图服务仍被拒绝，节点正常退出。
- 统一 launch 与旧 `mapping.launch.py` 的定位配置在缺少 `map_path` 时实际启动返回退出码 1，报错发生在节点启动之前；配置单元测试还覆盖定位 replay、无效/空地图路径和输出保护。
- 已修正逐帧地图累积和手动/退出保存路径，但没有真实传感器输入，因此完整实机场景建图及其 Ctrl+C 最终 PCD 内容尚未验证。文件格式写入链路由原生测试覆盖。

部署资料与打包回归（前次交付快照）：

- 工作区根目录新增正式 `dds_config/`、`doc/`、`docker/`；原电脑外层四个目录同步现行配置和兼容脚本，旧版本保留在 `reference/`，不进入部署 ZIP。
- Bash、Zsh 及旧 x86/ARM 环境入口在实际 Humble 容器内正确加载本工作区；默认 Fast DDS/domain 18，指定 Cyclone/domain 23 后 URI 正确指向新工作区 XML。
- 工具测试在本地假 Docker 下覆盖自动/显式容器选择、嵌套 bind mount、含空格路径和参数边界、歧义拒绝、DDS XML 和环境片段、Docker 入口与打包目标保护。
- 旧定位 shell 入口实际缺少 `map_path` 时返回退出码 1；旧配置网卡脚本不再修改网络。
- 候选部署 ZIP 约 23 MiB，266 个条目，CRC、必需源码、第三方实现、两套 SDK ELF 架构和关键脚本执行权限检查通过；不含 x86 编译缓存、运行日志、录包或绝对路径 symlink。
- 候选 ZIP 用 Linux unzip 在宿主机解压，复制到容器全新路径 `/tmp/fastlio-release-check.ZXtxbl/fastlio-mid360_space` 后完成无缓存 Release 构建（两个包，3 分 11 秒）。日志明确使用该目录的 SDK / driver，随后 11 个 CTest 和三种无硬件启停自检全部通过；不存在对原电脑源码或编译产物的构建依赖。
- 前次交付包只在候选基础上更新说明和验证记录，构建源码与脚本相同；新镜像及 Jetson ARM64/实机链路仍属于待现场验证项。

Git 规则与本地配置回归：

- 在项目根目录补齐分类 `.gitignore` 与 `.gitattributes`；真实 `git check-ignore` / `git check-attr` 检查 SDK、默认配置、两个包清单、许可证和测试地图不被忽略，数据/本地配置被忽略，源码 LF 与 SDK binary 属性正确。测试 Git 元数据仅位于自己的临时目录，项目没有执行 git init / commit。
- driver 子目录不再忽略 `package.xml`；已平铺的 ikd-Tree 保留普通源码，其旧嵌套 `.gitmodules` 被忽略而未删除。
- 生成 `config/local/MID360.local.json` 保留默认 IP；默认 JSON SHA256 修改前后均为 `d4d04a63f0c4e6035463444e7b0d20215546fb9298c7c6d8b5be1807a1f9e38e`。生成器校验 IPv4/结构、联动 IP、保留端口/参数，独占创建拒绝覆盖已有配置。
- 统一 launch 验证显式相对本地 JSON/YAML 路径，缺失/非法 JSON 被拒绝，本地定位 YAML 仍要求 map_path。建图启动自检实际调用 init_local_config 和 run.sh，用临时本地回环 JSON 初始化 SDK，driver / FAST-LIO 正常退出；不涉及真实雷达命令。
- 更新的工程通过增量 Release 构建、12 个 CTest 和三类 ROS 无硬件启动测试；Git / Docker 上下文 / ZIP 排除本地配置。临时打包检查 274 个条目、CRC/必需文件/SDK ELF/脚本权限均通过，没有携带现有 config/local 副本。
- 此次未覆盖前次交付 ZIP；要把当前 Git/本地配置功能拷到 Jetson，需运行 scripts/package.sh 生成新的部署包。本地机器配置需在 Jetson 重新生成。

尚未完成/环境限制：

- ARM64 的 SDK 路径、ELF 指令集以及 driver 所需 SDK API 已核验，但没有 ARM64 原生环境，未进行 ARM64 编译或运行。
- 当前容器没有默认配置要求的 `192.168.123.18`，因此实机网络检查正确返回失败。本次未改变主机网卡、路由或防火墙，未进行真实雷达数据接收/建图精度测试。
- 未验证 RViz GUI；默认无 GUI 自检不依赖显示环境。
- 新的 PC/Jetson 通用 Dockerfile 已核对官方 Humble multiarch tag 和 ROS 包名，但未构建这个新镜像；当前验证使用既有容器。Cyclone DDS 配置与环境加载通过检查，既有容器未安装该 RMW，未进行真实 Cyclone 跨机通信测试。
- rosdep 数据库未初始化，rosdep check 不能执行；实际 CMake 编译与节点启动所需依赖均已找到，未安装额外软件。
- 构建期间出现 bind mount 的亚秒级 clock-skew 警告，以及上游 Boost bind 弃用提示；构建成功，最终目标文件晚于源码，旧 `SigHandle` 符号已消失，运行自检通过。

复现入口：`bash scripts/build.sh`、`bash scripts/test.sh`、`bash scripts/check_network.sh`。详细用法见 [README.md](README.md)，节点启动和退出日志见 `log/smoke/`。
