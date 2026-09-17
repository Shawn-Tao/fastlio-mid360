# 历史入口备份

这些文件只是保留旧名字的兼容 wrapper，不再属于常用入口。移动后已调整相对
路径，仍可从工作区执行 `bash scripts/backup/run_mapping_local.sh ...` 等命令。
不要直接搬回顶层；如要恢复，应同时恢复相对路径。

| 备份 | 现行入口 |
|---|---|
| run_mapping_local.sh | `bash scripts/run.sh mapping ...` |
| run_localization_local.sh | `bash scripts/run.sh localization map_path:=...` |
| config_interface.sh | `bash scripts/check_network.sh [本地 JSON]`（只读） |
| x86_setenv.sh | `source scripts/setenv.bash` 或 `source scripts/setenv.zsh` |
| arm64_setenv.zsh | `source scripts/setenv.zsh` |

架构检测由 CMake 完成，不再需要按 x86/ARM 选择环境脚本。备份保留在源码仓库，
但部署 ZIP、Docker 构建上下文不携带它们；所有现行入口均不依赖 backup/。
