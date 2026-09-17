# Git 版本管理约定

以 `fastlio-mid360_space/` 为唯一仓库根目录，两个 ROS 包和已复制的第三方源码作为
普通目录提交；不依赖旧 FAST_LIO / driver 仓库或 Git 子模块。
源副本遗留的 `src/FAST_LIO/.gitmodules` 被忽略，不代表实际子模块；其源码仍跟踪。
driver 的旧 `package.xml` 忽略规则已去除，两份包清单都必须提交。

## 跟踪范围

- 跟踪源码、默认 ROS/雷达/DDS 参数、launch、文档、Docker、脚本、测试、许可证。
- 跟踪两套 SDK 的头文件与 `.so` / `.a`，以及 `pcd_map/test.pcd`。
- 可以跟踪地图清单 YAML / JSON / SHA256，记录地图文件、代码 commit、标定和初始位姿。
- 不跟踪编译产物、运行日志、录包、GT CSV、场景点云、Python / IDE 缓存、ZIP。
  轨迹默认 `records/` 被整体忽略；地图 `.pcd.json` 可跟踪，须和对应 PCD 配对保管。
- 不跟踪 `config/local/`、`*.local.json/yaml/yml/xml/sh/bash/zsh`、私有 `.env` 和 secrets。
- `.env.example` 与 `.env.*.example` 可以跟踪；不要把实际口令写入模板。
- `maps/` 与 `reference/` 是旧历史资料，不进入新仓库或部署 ZIP。

`.gitignore` 避免全局忽略 `.so`、`.a`、JSON/YAML/PDF，防止误删部署必需文件。
忽略规则由临时独立 Git 元数据目录中的真实 `git check-ignore` 回归测试验证，
不会在项目中自动初始化仓库。注意 `.gitignore` 不会停止跟踪已经提交的文件，
如需处理已跟踪的本地副本，要明确检查后单独移出索引，保留磁盘副本。

## 换行与大文件

`.gitattributes` 将脚本、源码、CMake、ROS 接口和文本配置的 Git checkout 统一为 LF；
SDK、PCD、PDF 等使用 binary 属性，不进行文本转换、文本合并或普通文本 diff。
添加属性不会立即重写磁盘上的源码换行。
两套 SDK 默认使用普通 Git，没有引入 LFS 依赖；若以后启用 LFS，先确认远端支持，
clone 后、编译及 ZIP 打包前执行 `git lfs pull`，确保真实二进制已下载。
Linux clone / unzip 要保留脚本执行权限；Windows 中转导致权限变化时检查后修复。

## 本地配置

实机 IP 不改默认 JSON，在当前机器重新生成本地副本：

```bash
bash scripts/init_local_config.sh --name jetson --host-ip 192.168.123.18 --lidar-ip 192.168.123.114
bash scripts/check_network.sh config/local/MID360.jetson.local.json
bash scripts/run.sh mapping lidar_config:=config/local/MID360.jetson.local.json map_name:=lab_a
```

这些 IP 只是示例，应换成目标机真实地址。初始化不需要 ROS/Docker/sudo，不覆盖已有
本地文件、不改默认 JSON、不配置网卡；配置选择保持显式。详细用法见
[config/README.md](../config/README.md)。本地副本不会随 Git / ZIP 迁移到 Jetson。

## 首次初始化（由用户手动执行）

```bash
git init -b main
# 如本机尚未配置身份，按实际值配置本仓库，不改变全局设置：
git config user.name "Your Name"
git config user.email "you@example.com"
git add .
git status --short
git diff --cached --stat
git commit -m "Initial unified Humble workspace"
```

提交前确认 SDK、两个 package.xml、ikd-Tree / IKFoM 都在，且日志/本地配置不在。
当前工具不会初始化项目仓库、创建 commit、添加 remote 或推送。
选择远端时先确认 SDK 等依赖的许可证与再分发要求，保留第三方许可证。

现场验收后可手动创建标签，并在地图/实验记录中写入该 commit；工作目录存在未提交
改动时也应记录，避免把未提交代码的测试结果误认为某个标签的结果。

官方参考：[Git ignore](https://git-scm.com/docs/gitignore)、
[Git attributes](https://git-scm.com/docs/gitattributes)。
