# DDS 配置：PC / Jetson

DDS 是 ROS 节点之间的通信；MID360 的 UDP 收包地址仍由
`src/livox_ros_driver2/config/MID360.json` 配置。二者不能互相替代。

快速脚本默认 Fast DDS、`ROS_DOMAIN_ID=18`，不固定网卡；无硬件自检独立使用
localhost 域 91。不要把 `ROS_LOCALHOST_ONLY=1` 留在跨机运行终端。

选择 Cyclone DDS（两端需安装 `ros-humble-rmw-cyclonedds-cpp`）：

```bash
export FASTLIO_DDS=cyclone ROS_DOMAIN_ID=18
# PC 如需使用单独配置，在 source 前指定绝对路径：
# export CYCLONEDDS_URI="file://$PWD/dds_config/cyclonedds_pc.xml"
source scripts/setenv.bash           # Zsh 改为 scripts/setenv.zsh
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
