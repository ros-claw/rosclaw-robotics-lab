# Luna 视觉导航

当前入口用于 P0 仓库基线。它运行一个 Isaac Sim 6.2 世界和一套隔离的 Nav2 Humble，检查新鲜的仿真时钟、传感器数据、TF 和生命周期状态。启动时不发送移动目标。

## 环境

- Ubuntu 22.04 x86_64，宿主 ROS Humble，Isaac 使用安装包自带的 Python 3.12 和 Humble bridge。
- Nav2 在单独的容器中运行。容器复用宿主网络与 IPC，使用与 Isaac 相同的 UID/GID 和本地 Fast DDS 域。
- 使用 NVIDIA `IsaacSim-6.2.0` 的 Nova Carter 仓库场景、地图和 DWB 基线参数。
- 默认 Isaac 目录为 `/data/omniverse/isaac-sim-6.2.0`。本次验证使用 GPU 1。可以通过 `--gpu`、`--domain`、`--timeout` 覆盖运行参数。

## 复现 P0

从本目录运行。需要 Docker、宿主 Humble 的 Python ROS 包、Python 3 的 PyYAML 和 NVIDIA Isaac Sim 6.2。当前工作区已准备以下 NVIDIA 源码与镜像。

```bash
docker build -f config/Dockerfile -t rosclaw/luna-nav-humble:p0 config
python3 scripts/lab.py doctor --output /data/rosclaw/robotics_lab/p0-evidence/environment.json
python3 scripts/lab.py start
```

`start` 在前台维持本次运行。收到 `READY` 时，就绪详情已写入该次运行的 `readiness.json`。在另一个终端从同一目录停止。

```bash
python3 scripts/lab.py stop
```

P0 的全量复位是先停止，再启动一个新世界和新的取证目录。失败会返回非零状态并保留日志。退出后有 `stop.json`、`container-stop.json` 和 `result.json`。默认取证路径位于本 Git 仓库的上一级，可通过 `LUNA_EVIDENCE` 指定。路径和环境覆盖见 [runtime.env.example](config/runtime.env.example)。

当前镜像基于本机已有的 `rosclaw/ros2-humble-bridge:latest`。NVIDIA 工作区需与上游 [IsaacSim-6.2.0](https://github.com/isaac-sim/IsaacSim-ros_workspaces/tree/IsaacSim-6.2.0) 标签匹配，Git 提交为 `f215318fb48913d30b938259b2163dfcf4df0caf`。如需将工作区放在别处，设置 `LUNA_ISAAC_ROS_WS`。SimScene checkout 的路径由 `LUNA_SIMSCENE` 指定，P0 读取其提交 SHA。

## RGB-D 取证

使用 `python3 scripts/lab.py start --rgbd` 启用前向相机。出现 `READY` 后从另一个终端运行 `python3 scripts/lab.py observe`。采集输出包含原始 RGB PNG、米制深度 NPY、相机标定、图像时刻的 TF 与哈希。标定与采样详情见 [P1 报告](P1_REPORT.md) 和 [传感器契约](SENSOR_CONTRACT.json)。
