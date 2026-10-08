# 在 DGX Spark 上复现真实 Native Agent 仓库巡检

本机配置为 Ubuntu 24.04 aarch64、GB10、驱动 580.159.03、Isaac Sim
6.1.0 和 ROS 2 Jazzy。Isaac 在宿主机运行，Nav2 在 ARM64 容器运行。
不需要替换宿主机驱动或安装另一套宿主机 ROS。独立工程师在干净环境中的
复现仍待完成；本机验证不能代替这一步。

## 1. 准备依赖

准备 Docker（当前用户有使用权限）、Git、Node.js >=22、npm、Python 3 和
[uv](https://docs.astral.sh/uv/)。从 NVIDIA 下载 Isaac Sim 6.1.0 Linux aarch64
独立安装包，自行阅读并同意 EULA 后，解压到 `~/isaacsim`，运行
`post_install.sh` 和随包兼容性检查。不能在 Spark 上使用 x86 安装包。
本机安装包 MD5 为 `02824f6b3c20d941ab81bf33d027d40b`。
需要能够访问 NVIDIA 在线资产、GitHub、ROS 软件源及已有模型服务。

```bash
git clone https://github.com/ros-claw/rosclaw-robotics-lab.git
cd rosclaw-robotics-lab/challenges/01-isaac-warehouse-patrol
./scripts/setup.sh
./scripts/doctor.sh
```

Setup 从公开 Dockerfile 构建 ARM64 镜像，构建三个 NVIDIA ROS 包，并安装
独立的固定版本 ROSClaw。默认路径、镜像和 ROS 域在 `config/runtime.env`，
均可通过环境变量覆盖。已有 checkout 版本不一致时会退出，不会覆盖。
NVIDIA workspace 固定为 `a9e8471ee901bc2332c1e4aca94ac580713ca3ab`，
ROSClaw 固定为 `21838614bb14c39599b8acef731b2b64dad3b92a`。
Docker 基础镜像固定 digest，但 apt 包版本仍可能更新；每轮清单记录实际版本。

先使用固定 ROSClaw checkout 的正常模型配置流程，设置并认证一个能工作的模型。
验收程序复制现有 `~/.rosclaw/agent` 配置到私有隔离目录，不切换模型供应商。
本机真实会话使用已有 `openai-codex/gpt-6.1-sol`。认证文件不会进入公开报告。

## 2. 启动场景和 ROS 观测

```bash
ROSCLAW_CAMERA_VIEW=top ROSCLAW_CAPTURE_SECONDS=1400 ./scripts/demo.sh streaming patrol
```

启动最多等待 20 分钟，以加载在线资产、编译着色器并取得独立 PhysX 状态，
随后等待 Nav2 ACTIVE 和 NavigateToPose 就绪。输出中会打印绝对证据目录，
将它原样填到下一步的 `PHYSICS_DIR`。启动程序不会自动发导航目标。
可将 streaming 改为 headless 或 gui；本机真实渲染截图已验证，外部 WebRTC
客户端连通性尚未验证。显示相机可选 official、top、overview、follow。
项目使用 ROS 域 61 和本机 rosbridge 19091 端口；若域已被占用，请整体改用空闲域。

在另一终端持续运行：

```bash
./scripts/start-agent-observers.sh
```

为解决实际加载时的循环 payload 引用，四个非必要 Hawk ROS variant 在会话层禁用。
LiDAR 保持开启，NVIDIA 原始 USD 未修改。巡检 Nav2 配置与官方基线分开保存，
具体参数变化和原因见 [接入说明](patrol-integration.md)。

## 3. 输入一句任务

每次重置必须使用一个新目录，放在**所有 Git 仓库之外**。聊天 CLI 会发现父级
Git workspace；在此仓库内放运行目录可能意外共享 workspace。

```bash
PHYSICS_DIR=/启动程序打印的绝对目录
RUN_DIR="$HOME/sim/rosclaw-isaac-runs/$(date -u +%Y%m%dT%H%M%SZ)"
TASK='请先检查机器人状态，然后依次巡检仓库入口、货架区和仓库通道，遇到障碍物自主避让，完成后返回 Home，并保存巡检报告和记忆。每个位置停留检查实际位姿与激光数据。'
./scripts/run-native-acceptance.sh "$RUN_DIR" "$PHYSICS_DIR" "$TASK" entry shelf aisle home
```

验收程序向真实 Native chat 发送一次任务，测试操作员只批准隔离的、绑定 Body 的
SIM 授权卡。末尾站点参数是独立评价器的顺序合同，不会作为路线发送给 Nav2。
Agent 自行观测 ROS，逐站提出单个 `navigation.navigate_to_pose`，最后调用
`patrol.verify_and_remember`。物理动作经过 Agentd、operatord、rosclawd 和规范收据。
MCP 工具没有直接运动执行权，Agent 不创建 Runtime 或注册 executor。

改变任务顺序时，重新启动场景，同时修改自然语言和验收合同，例如先通道、
再入口、再货架、最后 Home，对应 `aisle entry shelf home`。所有三站和返回仍须完成。
站点由实际地图和组合 USD 量取，执行器只接受注册 ID，不接受任意坐标。

## 4. 独立验收和清理

```bash
python3 evaluator/run_report.py --directory "$RUN_DIR" --output reports/my-native-run.json
./scripts/stop.sh
```

每站必须满足：Nav2 状态 4/错误码 0、PhysX 位置误差不超过 0.4 米、朝向误差
不超过 0.35 弧度、稳定停留至少 2 仿真秒、新鲜有效 LiDAR、完整接触观测且
非地面接触为零。收据 Body 哈希及证据 SHA-256 必须一致。现有 Memory 必须
保存成功验证经验，TaskKernel 只能在 Memory 收据完成之后关闭。缺失观测不能通过。
每目标恢复预算最多六项 Nav2 操作、最长 420 墙钟秒；取消和超时不能写成成功。

原始轨迹、私有模型目录及终端日志保留在 Git 外。**不要上传整个运行目录**，
里面有模型认证和操作员密钥。公开摘要不含原始模型思考。失败证据与成功证据一起保留。
stop 只清理带项目标签的容器和经过 PID/出生时间核验的 Isaac 进程；先等待 Native
验收完成自身清理，再停止仿真。Ctrl-C 验收程序会请求取消并进入清理。

## 5. 基线、故障实验和视频

新重置后可运行 `./scripts/smoke-official.sh`，这是 NVIDIA 官方三目标基线，
不能当成 Agent 验收。默认配置有一站超出 0.4 米；校准基线误差为
0.146 / 0.174 / 0.079 米。

`ROSCLAW_CONTACT_TEST=1` 用于独立碰撞正控制，正式巡检时不能开启。
`ROSCLAW_OBSTACLE_TEST=1` 可加入会话层未映射静态箱体，默认关闭，用于单独障碍实验。
ROS 容器中的只读 `ros2/obstacle_witness.py` 记录真实点云、代价地图和规划路径。
未经实测的障碍实验不能宣称通过。

`visualization/render_video.py` 组合实际 Isaac 截图、PhysX 和带时间戳的真实终端输出。
80 秒宣传模式明确标注时间压缩。教程模式保留全部录制墙钟时间，以 2 fps 重建；
这是逐帧重建，不是连续屏幕录制。渲染需要 Pillow、PyYAML、pyte、Noto CJK/
DejaVu 字体、官方地图 PNG/YAML 和 ffmpeg。地图/资产不随此仓库分发。

从 Releases 下载公开证据包后，可在没有 Isaac 或模型的机器上复核记录：

```bash
python3 evaluator/replay_archive.py native-acceptance-2-evidence.zip
```

这只核验已录制的收据、哈希和物理证据，不等于重新运行仿真。在启动时选择固定
显示视角；运行中切换投影曾使 Isaac 停顿，因此该功能已移除。验证过的俯视位于屋顶下方。

### 复现带独立验证门槛的未映射箱体实验

先停止上一场景，在新场景启动命令前增加 `ROSCLAW_OBSTACLE_TEST=1`。
第二终端照常启动 bridge/probe。第三终端先记录真实点云、代价地图和路径：

```bash
EVIDENCE_NAME=$(basename "$PHYSICS_DIR")
ROSCLAW_CONTAINER_NAME=rosclaw-warehouse-path-witness ./scripts/ros-container.sh \
  python3 /lab/ros2/obstacle_witness.py \
  --output "/lab/reports/runs/$EVIDENCE_NAME/path-witness.jsonl" \
  --ros-args -p use_sim_time:=true
```

换一个私有运行目录，任务中明确说明箱体，并要求“通道→入口→货架→Home”：

```bash
ROSCLAW_REQUIRE_OBSTACLE_EVIDENCE=1 ./scripts/run-native-acceptance.sh \
  "$RUN_DIR" "$PHYSICS_DIR" "$TASK" aisle entry shelf home
```

额外环境变量只增加验收合同，不给 Agent 注入导航程序。在写入成功 Memory 前，
必须有真实点云命中、局部主代价地图占据单元、保守足迹净距和零接触。地图空闲像素
证明绑定实际官方地图文件哈希；到点本身不足以通过。首次不充分的结果与修复后的
完整结果都保留。使用 `visualization/package_evidence.py` 仅打包白名单证据，离线
重放也会核验这个额外门槛。
