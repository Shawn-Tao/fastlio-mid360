# PCD 地图目录（与 src/ 同级）

## 常用指令

NX 工作区根目录建图，显式使用本机雷达配置：

```bash
bash scripts/run.sh mapping lidar_config:=config/local/MID360.jetson.local.json \
  map_name:=lab_a publish_map:=true
```

正常 Ctrl+C，等待最终保存/退出；保留生成的 PCD 和同名 `.pcd.json`。
定位选择实际地图并建议严格校验：

```bash
bash scripts/run.sh localization lidar_config:=config/local/MID360.jetson.local.json \
  map_path:=pcd_map/实际地图.pcd search_radius:=3.0 map_metadata:=strict
```

`strict` 必须有合法配套 JSON；无 JSON 的旧地图可不传此项，默认 auto 警告兼容。
编译不需要地图；AGX 看图也无需复制 PCD，分别用 `bash scripts/rviz.sh mapping` /
`localization`。编译和首次配置见 [主文档](../README.md)。

## 命名、保存和元数据

建图默认保存为 `<启动时间戳>_<map_name>.pcd`，默认名称 `map`。
例如 `20260916_160000_123456_lab_a.pcd`（时间戳使用节点所在容器/系统时区）。
同次运行的 `/map_save` 和 Ctrl+C 会更新同一文件，不会自动覆盖之前运行生成的地图。
配套 `<文件名>.pcd.json` 保存标定/过滤/参考重力、完整性、点数及 CRC32。
默认 0.1 m 全局体素去重、2000000 点容量上限；容量不足标记不完整并拒绝验证定位。
默认存图层跨帧确认、保守自由空间清理；未确认候选不会导出，静止行人仍可能入图，
人离开后需要有效回波重复照过旧位置。不会修改实时 ikd-Tree 或定位参考 PCD。
关闭对照 static_filter:=false，参数/统计/边界见 [静态存图过滤](../doc/STATIC_MAP_FILTER.md)。
`/map_save` 成功只是快照入队，等 `/map_save/status` saved；默认每 60 s 检查点。
定位必须显式指定 `map_path:=...`，不自动选择最新地图，不默认加载 test.pcd。
正式定位建议 map_metadata:=strict；auto 兼容没有 JSON 的旧 PCD，但会警告。
外部修改/变换地图后不要沿用旧 JSON。细节见 [运行安全说明](../doc/RUNTIME_SAFETY.md)。

`test.pcd` 是复制的旧示例，仅用于启动自检，不代表当前场地。
