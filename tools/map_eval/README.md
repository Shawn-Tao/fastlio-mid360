# 本机离线点云验证环境（uv，CPU-only）

在工作区根目录执行。安装只写本工程本地目录，不需要 sudo，也不修改系统 Python 或 shell 配置。
这是 Linux x86_64 电脑环境，不用于 Jetson/ROS 编译；不安装 CUDA、ROS 或 CloudCompare。

```bash
bash tools/map_eval/setup.sh
bash tools/map_eval/run.sh python check_env.py --maps ../../../map_bak
```

run.sh 在子进程中使用独立环境，工作目录切换到 tools/map_eval；相对地图路径以该目录为基准。
`check_env.py` 仅检查解释器隔离、依赖、CPU 运算、无界面绘图和 PCD 读取，不给地图质量打分。
原 PCD/JSON 始终只读。环境配置不依赖 git commit；提交版本只由用户执行。

## 哪些文件进入版本管理

- 纳入 Git：pyproject.toml、uv.lock、.python-version、COLCON_IGNORE、工具源码和本说明。
- 不纳入 Git、部署 ZIP 或 Docker 上下文：根目录 .local_tools/，任何 .venv/、venv/、.uv-cache/，
  以及 tools/map_eval/results/ 中的本机报告。
- .local_tools/ 内保存 uv 可执行文件、独立 Python、下载包、依赖缓存、Python/Matplotlib 缓存。
  环境本体在 tools/map_eval/.venv/；不复制到另一台机器、不自动写入 shell 启动配置。
- 不使用 sudo pip、pip --user、--system-site-packages、uv pip install --system，也不修改 /usr/bin/python3。

依赖固定 Python 3.12 范围及 Open3D CPU 0.19.0；首次安装生成 uv.lock，此后按锁文件重建。
普通运行使用 --locked，依赖声明与锁不一致时报错，不会悄悄更新锁文件。
如需调整依赖，应显式更新 pyproject.toml 并重新生成/检查锁文件，由用户决定是否提交。
uv/Python 安装需要网络；共享库若缺失须另行确认，脚本不会自动 apt 安装系统包。

## 日常调用

无需 source/activate。run.sh 清除子进程继承的 PYTHONHOME/PYTHONPATH/VIRTUAL_ENV，
强制使用受管理解释器和本工具 .venv，不改父终端中的 ROS Python/DDS 环境。

```bash
bash tools/map_eval/run.sh python -c "import sys, open3d; print(sys.executable, open3d.__version__)"
```

只复制源码、依赖清单和锁文件即可复现；执行 setup.sh 重建本机环境。由 COLCON_IGNORE
确保离线工具不参与 driver/FAST-LIO 的 colcon 编译，也不会改变已有地图/雷达配置。
