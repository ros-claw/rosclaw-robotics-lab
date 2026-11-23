# P0 环境与运行证据

测量日期为 2026-11-21（Asia/Shanghai）。原始 JSON 中的尝试 ID 使用 UTC。

| 项目 | 本机结果 |
| --- | --- |
| 操作系统 | Ubuntu 22.04.5 LTS，x86_64 |
| GPU | 4 × NVIDIA RTX A6000，驱动 595.91.07。此次选择 GPU 1 |
| Isaac Sim | `6.2.0-rc.14+release.52805.02189a84.gl`，Python 3.12.14 |
| ROS | 宿主 Humble Python 3.10，Isaac 自带 Humble bridge 5.1.4，Nav2 容器 Humble 1.1.20 |
| 基础项目 | Robotics Lab `545830cbd2ea8fd20fb579edb113822f7d3c9841`，SimScene `3660dc1b37610d7b171934dd86ff67df01107d50` |
| NVIDIA 工作区 | `f215318fb48913d30b938259b2163dfcf4df0caf`（IsaacSim-6.2.0） |
| 资产 | 安装包 us 配置的 HTTPS 6.2 根，Nova Carter 官方仓库场景请求返回 200 |
| Luna | 预设 `gpt-6-luna`。本次 shell 未设置 `OPENAI_API_KEY` 或 `OPENAI_BASE_URL`。P0 不调用模型 |

宿主无 Nav2，软件包解析报告 `liblcms2-dev` 与已安装的 `liblcms2-2` 版本冲突。将 Nav2 装入独立容器。现场有另一个长期运行的 Isaac 6.0.1 实例，本阶段的停止操作只针对该入口创建的进程组和容器。

## 实际运行

从 challenge 目录运行 `docker build -f config/Dockerfile -t rosclaw/luna-nav-humble:p0 config`。采集命令为 `python3 scripts/lab.py doctor --output /data/rosclaw/robotics_lab/p0-evidence/environment.json`。成功启动命令为 `python3 scripts/lab.py start --domain 189 --timeout 150`。

成功试次 `20261120T233722Z-28075e5e` 以真实 DDS 样本通过就绪检查。检查耗时 23.793 秒。其间观察到 88 条推进的时钟消息、88 条里程计、10 个推进的 LiDAR 点云及 12 条推进的 LaserScan。检测到 `map ← base_link`、`odom ← base_link`、`base_link ← front_3d_lidar`，AMCL、规划器和控制器均为 active。入口未发移动目标。显式执行 stop 后 Isaac 返回 0，Nav2 容器被确认移除。Docker 客户端超过 10 秒未退出，由监督进程终止。另一个 Isaac 进程仍在。

取证目录 `/data/rosclaw/robotics_lab/p0-evidence`。各次尝试下的 `manifest.json` 记录实际 argv 和 DDS 域，`environment.json` 记录版本和仓库，`readiness.json` 记录就绪检查，`isaac.log` 和 `nav2.log` 保留原始输出。

| UTC 试次 | 结果 |
| --- | --- |
| `probe` | 全量 Kit 入口试探，启动应用并加载 USD，未通过 DDS 就绪检查，结束该进程组 |
| `20261120T232651Z-b4d1e4da` | 8 秒超时负测，返回 1，清理进程和容器 |
| `20261120T232740Z-94b6103f` | 全量 Kit 等待 viewport handle，停止 |
| `20261120T232946Z-ca503e95` | 全量 Kit 执行场景 hook，未持续产出时钟及 TF，停止 |
| `20261120T233135Z-5771bad6` | 独立 SimulationApp 启动，未通过就绪检查 |
| `20261120T233404Z-66f2e76b` | RaytracedLighting 驱动宿主时钟和 odom，容器未得到所需 TF |
| `20261120T233541Z-14ae6d67` | UDP-only 配置未发现样本，启用 motion BVH 的场景更新发生阻塞 |
| `20261120T233722Z-28075e5e` | RaytracedLighting，Nav2 使用同 UID/GID 及共享 IPC，真实 DDS 就绪 |

P0 保留了仓库静态基线。RTX 日志提示 LiDAR 未使用 motion BVH，针对运动中的点云畸变仍需在 P3 验证。此前的一次 BVH 开启尝试没有通过首帧更新。当前的静止就绪结果只覆盖启动和消息传输。

## 文件与检查

本阶段新增 `scripts/lab.py`、`scripts/readiness.py`、两个 `isaac/` 基线模块、容器与运行配置和回归测试。原实施计划已复制到仓库指定的 `docs/superpowers/plans` 路径并校验哈希。

使用 CI 固定的 ROSClaw 提交 `9862058c445c88156b9080832b5dd803c75279ae` 和 Python 3.12 运行三个 challenge 的测试，72 项通过。当前 P0 的就绪和停止证据独立于 Python 单元测试。
