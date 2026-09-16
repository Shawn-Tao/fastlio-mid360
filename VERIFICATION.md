# 验证记录（2026-09-16）

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
