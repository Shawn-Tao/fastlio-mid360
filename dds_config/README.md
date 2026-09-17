# DDS 配置：PC / Jetson

## 常用指令：同一局域网的两端

在各自工作区根目录执行；默认 Fast DDS、`ROS_DOMAIN_ID=18`，无需额外 export。

NX 建图（已编译、已创建本机雷达配置）：

```bash
bash scripts/run.sh mapping \
  lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=lab_a publish_map:=true
```

AGX 看图（已有 Humble、RViz2 和桌面，无需编译本工作区）：

```bash
bash scripts/rviz.sh mapping
```

定位端/定位显示分别改用 `run.sh localization` / `rviz.sh localization`，
完整命令见 [主文档速查](../README.md)。若更换 domain，两端一致，例如：

```bash
# NX：仅在不运行另一套计算端时执行
ROS_DOMAIN_ID=23 bash scripts/run.sh mapping \
  lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=lab_a publish_map:=true
```

```bash
# AGX：与上述 NX 使用同一个 domain
ROS_DOMAIN_ID=23 bash scripts/rviz.sh mapping
```

跨机不能设置 `ROS_LOCALHOST_ONLY=1`。切换 RMW/domain 后，在各自相同 ROS 环境
的另一个终端检查（Bash；Docker 场景先进入对应容器）。交互终端也须设置与运行
命令一致的覆盖值，如 `export ROS_DOMAIN_ID=23` 后再 source。统一入口自动识别 Bash/Zsh：

```bash
source scripts/env.sh
ros2 daemon stop
ros2 node list
```

## 配置原理与可选 Cyclone DDS

DDS 是 ROS 节点之间的通信；MID360 的 UDP 收包地址仍由
`lidar_config:=...` 选择的 JSON 配置，未指定时才用默认 `MID360.json`。
二者不能互相替代；NX 雷达接收地址不要改成 AGX 的 IP。

快速脚本默认 Fast DDS、`ROS_DOMAIN_ID=18`，不固定网卡；无硬件自检独立使用
localhost 域 91。不要把 `ROS_LOCALHOST_ONLY=1` 留在跨机运行终端。

计算端统一用 `bash scripts/run.sh mapping|localization ...`；AGX/PC 独立显示用
`bash scripts/rviz.sh mapping|localization`，它自动加载同一 DDS 默认值，不启动
另一套算法/driver。建图远程显示须在计算端传 publish_map:=true；具体流程见
[脚本说明](../scripts/README.md)。

选择 Cyclone DDS（两端需安装 `ros-humble-rmw-cyclonedds-cpp`）：

```bash
export FASTLIO_DDS=cyclone ROS_DOMAIN_ID=18
# PC 如需使用单独配置，在 source 前指定绝对路径：
# export CYCLONEDDS_URI="file://$PWD/dds_config/cyclonedds_pc.xml"
source scripts/env.sh               # 自动适配 Bash/Zsh
```

XML 使用 `Domain Id="any"`，由 `ROS_DOMAIN_ID` 统一指定域。默认自动选择网卡；
多网卡时用 `ip -br addr` 找到 **PC ↔ Jetson 的 ROS 通信网卡**，把
`<NetworkInterface autodetermine="true"/>` 换成实际 `name="..."` 或
`address="本机网卡IPv4"`。不要填雷达自身地址。

若网络限制组播，在两个 XML 的 `Discovery` 中分别添加真实对端 IP 的 `Peers`；
没有固定启用任何示例 IP。两端保持相同 domain、兼容 QoS；为减少排查变量，
建议使用同一种 RMW，但不同 DDS 实现并非原理上不能互通。
切换 RMW/domain 后先 `ros2 daemon stop`，再检查 `ros2 node list`。

宿主机脚本会转发 DDS 环境变量到容器；自定义 `CYCLONEDDS_URI` 中的路径必须是
**容器内可访问的路径**。默认 URI 会在容器内按工作区位置生成，不要求固定挂载路径。

配置语法依据 [Cyclone DDS 0.10 配置 schema](https://github.com/eclipse-cyclonedds/cyclonedds/blob/releases/0.10.x/etc/cyclonedds.xsd)，
网卡选择参见 [官方说明](https://cyclonedds.io/docs/cyclonedds/latest/config/network_interfaces.html)。
