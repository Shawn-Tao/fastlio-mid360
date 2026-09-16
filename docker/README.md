# PC / Jetson：Humble 容器

三份 `Dockerfile`、`Dockerfile_pc`、`Dockerfile_jetson` 现在是相同的兼容入口。
统一使用 `ros:humble-ros-base-jammy`，由 Docker 在目标机器选择本机架构；
[官方镜像清单](https://github.com/docker-library/official-images/blob/master/library/ros)
列出此 tag 支持 amd64 与 arm64v8。不要在 Jetson 上直接使用旧 `osrf/ros` desktop 镜像。

此工程使用 CPU / PCL / OpenMP，没有 CUDA 依赖，因此默认不采用 L4T 镜像、
不启用 `--runtime=nvidia`，也不需要 `--privileged`。若后续添加 GPU 工程，
应另行匹配目标 JetPack / L4T / CUDA，不能把本镜像视为 GPU 兼容保证。

在解压后的工作区根目录执行：

```bash
bash docker/build.sh                 # 首次需要联网拉镜像及 apt 依赖
bash docker/run.sh --detach          # 创建/启动 fastlio-mid360 容器
FASTLIO_NATIVE=0 bash scripts/build.sh
FASTLIO_NATIVE=0 bash scripts/test.sh
FASTLIO_NATIVE=0 bash scripts/check_network.sh
FASTLIO_NATIVE=0 bash scripts/run.sh mapping map_name:=lab_a
# 交互终端（source scripts/setenv.bash 后可用 ROS CLI）：
bash docker/run.sh
```

固定挂载目标 `/workspace/fastlio-mid360_space`，源目录可以放在任意用户路径。
编译/运行脚本支持旧容器的其他挂载布局，按 bind mount 自动解析。
存在多个匹配容器时必须指定 `FASTLIO_CONTAINER=名字或ID`；不会自动挑一个。
`docker/run.sh` 不删除或替换已有容器，名字冲突且挂载/网络不一致时会报错。
改变镜像或 GUI 配置时请用新的容器名。

镜像默认采用执行 `docker/build.sh` 的用户 UID/GID，避免宿主机文件变成 root 所有；
建议由普通用户执行，不要用 sudo 跑 build 包装脚本。Docker 权限按目标机器正常配置。
所有依赖已在构建镜像时安装，容器用户默认没有 sudo。

默认无 RViz。需要在容器本机运行 GUI 时，构建和创建新容器时分别显式开启：

```bash
FASTLIO_INSTALL_RVIZ=1 bash docker/build.sh
FASTLIO_CONTAINER=fastlio-mid360-gui FASTLIO_DOCKER_GUI=1 bash docker/run.sh --detach
FASTLIO_CONTAINER=fastlio-mid360-gui bash scripts/run.sh mapping map_name:=lab_a rviz:=true
```

需要有效 `DISPLAY`、X11 socket 与目标显示服务器授权；脚本不调用 `xhost` 或修改授权。
Wayland / 无桌面环境建议 Jetson 只运行节点、PC 通过 DDS 使用 RViz。
Linux Jetson 使用 host 网络；Docker Desktop 的 host 网络不等于已经拥有物理雷达网卡。
必须让 `MID360.json` 接收地址在实际容器网络中存在，并检查 UDP 路由。

`.dockerignore` 排除源码部署不需要的 SDK/历史资料和运行数据，因为镜像只装环境，
源码、SDK、地图都在运行时挂载。没有自动 source 旧 overlay、替换 apt 镜像源或执行系统升级。
完整部署步骤见 [Jetson 部署指南](../doc/JETSON_DEPLOY.md)。
