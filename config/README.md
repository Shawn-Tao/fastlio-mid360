# 本地配置，不修改默认 IP

## 常用指令

在 NX 工作区根目录执行。首次创建本机副本，示例 IP 换成真实地址：

```bash
bash scripts/init_local_config.sh --name jetson \
  --host-ip 192.168.123.18 --lidar-ip 192.168.123.114
```

只有以上配置创建命令无需 ROS、Docker 或 sudo。`host-ip` 是 NX 接雷达的网卡地址，
不是 AGX 地址；拒绝覆盖已有副本，也不修改系统网卡。配置创建后做只读检查：

```bash
bash scripts/check_network.sh config/local/MID360.jetson.local.json
```

已编译且有可用 Humble/容器后，建图显式选择这个副本：

```bash
bash scripts/run.sh mapping \
  lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=lab_a publish_map:=true
```

建图正常退出后，定位同样显式选择（替换真实地图名）：

```bash
bash scripts/run.sh localization \
  lidar_config:=config/local/MID360.jetson.local.json \
  map_path:=pcd_map/实际地图.pcd search_radius:=3.0
```

AGX 只看图用 `bash scripts/rviz.sh mapping` / `localization`，不用 NX 的雷达 JSON；
完整跨机指令见 [主文档](../README.md)。

## 副本规则和高级配置

默认配置仍是已版本化的 `src/livox_ros_driver2/config/MID360.json`。
`config/local/` 用于每台机器的配置副本，全部被 Git / Docker 镜像上下文 / 部署 ZIP 忽略。
不会自动加载这里的配置，必须显式通过 `lidar_config:=...` 选择。

仅复制默认 IP、之后自行编辑可用 `bash scripts/init_local_config.sh`。
无 `--name` 的目标是 `config/local/MID360.local.json`。
初始化拒绝覆盖已有文件；后续直接编辑副本，或用新名字创建另一份。
生成器校验 IPv4 地址，并联动更新 SDK JSON 中的 `host_ip`、`lidar_ip` 和
`lidar_configs[].ip`，保留端口、点云格式及其他参数，不配置系统网卡。
`--template /路径/template.json` 可选用其他 MID360 模板；
`--output /路径/name.local.json` 可显式指定一个新目标，必须以 `.local.json` 结尾。
多雷达模板不能用单个 `--lidar-ip` 强行合并，需手动编辑副本。

快捷脚本固定 CWD 为工作区根目录，所以可直接使用相对路径；Docker 时副本位于
同一 bind mount 内，无需写死原电脑路径。手动 ros2 launch 的相对路径以当前 CWD 为准。
默认文件不会随着初始化或启动被修改；未传 `lidar_config` 时仍用默认 JSON。

FAST-LIO YAML / Cyclone DDS 也可复制到 `config/local/` 后调整：

```bash
cp src/FAST_LIO/config/mid360_localization.yaml config/local/mid360_localization.local.yaml
cp dds_config/cyclonedds.xml config/local/cyclonedds.local.xml
bash scripts/run.sh localization config_file:=config/local/mid360_localization.local.yaml \
  lidar_config:=config/local/MID360.jetson.local.json map_path:=pcd_map/实际地图.pcd
```

自定义 Cyclone 的 `CYCLONEDDS_URI` 使用运行环境内的绝对路径，详见
[DDS 说明](../dds_config/README.md)。本地 YAML 选择地图仍须传 `map_path`。
修改物理标定/过滤参数时，建图与定位应一致，不只是替换 IP。

本地配置不会随 Git clone 或部署 ZIP 迁移；在 Jetson 上重新初始化并按实际网络配置，
如需人工拷贝旧副本应单独确认，不要提交或覆盖默认配置。
