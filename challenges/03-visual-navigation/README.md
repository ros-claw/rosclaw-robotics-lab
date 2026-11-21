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

## 单分叉与受保护的 Nav2 控制测试

[灰盒报告](P2_REPORT.md) 说明如何使用固定版本 SimScene 导出两种换牌配方。已导出场景位于工作区的 `p2-evidence/pair-wall-signs`。受保护启动默认使用 Cyclone DDS。重新构建镜像会安装相应的 Humble RMW。

```bash
python3 scripts/lab.py start --protected --rgbd \
  --layout /data/rosclaw/robotics_lab/p2-evidence/pair-wall-signs/original
# 在另一个终端，READY 后发起显式控制测试
python3 scripts/lab.py controller-test c24
python3 scripts/lab.py stop
```

运行目录保留路径事件、PhysX 轨迹及接触对。`evaluator/controller_run.py` 可独立核对动作区间中的实际过门、到达和低速驻留。[架构](ARCHITECTURE.md) 记录当前运动所有权和传输边界。

## Luna 视觉探测

在本机提供 `OPENAI_API_KEY` 后，使用 `lab.py luna-probe` 读取已留存的 Isaac PNG 和中性场景清单。命令保存确切图像字节、哈希和经校验的决策。两种换牌结果和复验命令见 [P4 报告](P4_REPORT.md)。

## 运行一次 Luna 单分叉决策

在第二个终端配置好本机 `OPENAI_API_KEY`。在第一个终端启动 MPPI 灰盒，等待 `READY`。以下命令使用当前工作区已有导出。灰盒重新导出见 [P2 报告](P2_REPORT.md)。

```bash
export LUNA_EVIDENCE=/data/rosclaw/robotics_lab/p4-live-evidence/local
python3 scripts/lab.py start --protected --rgbd --record-overview \
  --layout /data/rosclaw/robotics_lab/p2-evidence/mppi/original
```

```bash
python3 scripts/lab.py luna-run
python3 scripts/lab.py stop
```

换牌时将布局改为 `/data/rosclaw/robotics_lab/p4-live-evidence/swapped-fixture`，重启新世界。模型提出通道，局部检查通过后执行完整路径。模型返回 `reobserve` 时，保持驻车、使用新 RGB-D 帧重新请求。默认最多三次，可通过 `--max-observations` 限制。返回 `hold`、额度耗尽或 API 失败时停留在原处。要复现 API 超时停车，另起一个新试次并运行 `python3 scripts/lab.py luna-run --api-timeout 0.05`，留出至少 3 秒采样后停止。小闭环在调用 API 前驻车。

运行结束后，从 `start` 的输出取得 `RUN`，从 `luna-run` 的输出取得 `MODEL_DIR` 和 `CONTROLLER_EVENTS`。`PUBLIC_MANIFEST` 是该场景导出的 `nav/public_scene_manifest.json`。宿主系统的 Python 需要 Pillow 和 NumPy，编码使用 FFmpeg。

```bash
python3 video/export.py "$RUN/overview"
python3 video/caption.py "$RUN/overview" "$MODEL_DIR" --events "$CONTROLLER_EVENTS"
python3 viewer/replay.py "$RUN" "$MODEL_DIR" "$CONTROLLER_EVENTS" "$PUBLIC_MANIFEST" \
  "$RUN/replay.html" --overview "$RUN/overview/isaac-live-1x.mp4"
# 对新建的超时试次独立复查 PhysX 轨迹
python3 evaluator/timeout.py "$MODEL_DIR"
```

录像采用独立斜俯视相机，原始采集帧与时间索引在 `RUN/overview`。FFmpeg 按实测墙钟帧间隔生成 1× 可变帧率视频。回放可离线查看车载模型输入、模型返回、路径验证、动作接受、PhysX 轨迹与录像。

实测试次、录像性能与待完成的阶段见 [运行结果](RESULTS.md)。


## 连续路径交接

同一灰盒支持同一路由版本内的两次路径接续。开始于原启动状态，在 `READY` 后运行一个命令。每条命令使用新启动的仿真世界。

```bash
# 用 Luna 选门，然后在同一通道内执行两次路径更新
python3 scripts/lab.py luna-run --replan
# 用明确的测试意图执行相同的更新，过门后取消活动路径
python3 scripts/lab.py controller-test c24 --replan --cancel-after-gate
# 独立复核控制试次
python3 evaluator/continuity.py "$RUN" "$CONTROLLER_EVENTS" "$PUBLIC_MANIFEST"
```

控制测试可省略 `--cancel-after-gate` 以完成整条路线。路径序列 2 持有一条实际 Nav2 规划结果，先提交序列 3，再投递序列 2 验证其被拒绝。轨迹与接触复核见 [P3 报告](P3_REPORT.md)。


### 复现标牌被运动卷帘遮挡

使用换牌场景启动仿真，在原有启动命令上加 `--visual-shutter-test`。卷帘按仿真时钟独立打开。出现 `READY` 后执行 `python3 scripts/lab.py luna-run --replan`。完整日志保留每帧的时间戳、哈希、模型回复和动作边界。回放命令加上 `--vision-session "$LUNA_SESSION"`，其中 `LUNA_SESSION` 是 `luna-run` 输出的 `luna-session-*.jsonl` 路径。

## 自适应任务的一键入口

`scripts/run_mission.py` 启动本次任务独占的 Isaac/Nav2，等待真实传感器就绪后执行任务、保存证据并停止进程。命令返回 0 表示经独立物理验收的到达，3 表示进入 HOLD，1 表示运行失败。具体状态在该次运行的 `mission-summary.json`。模型凭据仍从本机 `OPENAI_API_KEY` 读取。

```bash
python3 scripts/run_mission.py --layout /path/to/fork-layout --record-overview
```

入口默认启用自适应。先用新帧选择通道。行驶中依靠扫描、碰撞监控和滚动更新的代价地图。多个新地图样本确认跟踪路径受阻后，取消当前动作，确认 PhysX 低速驻留，再重验原通道。原通道重新通过规划、全局与局部代价地图检查后，直接恢复原语义承诺。此时保留路线版本，也无需重复询问 Luna。传感器失效时，停止当前任务。新模型结果仍经过当下的位姿、地图、可行路径和足迹检查。

可在任务 JSON 中使用 `task` 提供语义目标。默认仅允许保持原语义通道。若明确允许换路，在 `recovery_task` 声明替代条件。原路线的局部恢复预算耗尽后，控制器使用新帧、已筛除的旧通道和该条任务规则再次询问 Luna。模型请求上限、同一通道重试次数、总墙钟期限分别由 `--max-observations`、`--local-retries` 和 `--mission-time` 配置。地图检查预算见 `config/adaptive.yaml`。

实验可通过 `--scenario-obstacles scene/experiments/transient-blockage.json` 注入运动物体。该注入参数只传入 Isaac。临时、持续以及更早封堵的配方均位于 `scene/experiments`。三份配方使用交换标牌的灰盒，蓝色 B 位于左侧。允许绕行的实验另加 `--mission-policy scene/experiments/allow-shared-destination.json`。

使用 `python3 evaluator/adaptive_run.py "$RUN"` 复核封堵试次。生成回放时将对应的会话日志传给 `--vision-session` 并加 `--adaptive`，回放会关联所有控制段的路径、模型请求、停止证据与恢复决策。
