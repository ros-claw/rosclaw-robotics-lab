# ROSClaw Robotics Lab

**一句自然语言，让机器人完成有独立物理证据的仓库巡检。**

真实 ROSClaw Native Agent 在 DGX Spark 上，通过 ROS 2 Jazzy/Nav2 运行 Isaac Sim 6.1 官方仓库里的 Nova Carter。**v0.2 冻结源码回归：16 次尝试中（含启动失败） 14 次通过**，三类各至少五次独立 Native 测试；保留启动失败和模型请求中断，唯一一次标准追加测试已提前声明。成功整圈共 56 次到点记录，最大实际位置误差 0.151827 米。这是本机仿真工程验收，尚未证明生产可靠性或真实硬件能力。

**新增 Challenge 02：** 从 USD 已知货架生成动态观察位置，完成往返及真实 LiDAR 区域观测。开发与最终候选集成共 5 次尝试保留 1 次失败、4 次完整通过；[场景说明与独立三视角视频](challenges/02-semantic-inspection/README.md)。

## 场景任务 Challenges

| Challenge | 用户任务与能力 | 独立验收与当前状态 | 展示 |
|---|---|---|---|
| [01 · 仓库四站巡检](challenges/01-isaac-warehouse-patrol/README.md) | 一句话完成入口、货架、通道、返回；支持改序及未映射箱体 | v0.2：16 次尝试，14 次完整通过；每站物理到达/停留、LiDAR、接触及最终 Memory/TaskKernel | [180 秒宣传片](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.2.0/rosclaw-warehouse-promo-180s.mp4) · [510 秒教程](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.2.0/rosclaw-warehouse-tutorial-510s.mp4) |
| [02 · 语义观察点导航与区域观测](challenges/02-semantic-inspection/README.md) | 从 USD 已知货架语义生成新观察位置，再返回实际起点；不使用四站坐标 | 开发及最终候选集成共 5 次：1 次 FAIL、4 次完整 PASS（含三次 LiDAR 区域观测）；保留录制失败，非冻结版本可靠性统计 | [90 秒三视角展示](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/semantic-observation-v0.3.0/rosclaw-semantic-promo-90s.mp4) |

视频中的历史第五轮与 v0.2 冻结回归分别记录。每个 Challenge 页面说明输入、工具边界、成功门槛、失败和复现方式。

[![Synchronized three-view Native mission](challenges/01-isaac-warehouse-patrol/reports/v02-demo-poster.png)](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.2.0/rosclaw-warehouse-promo-180s.mp4)

[60s](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.2.0/rosclaw-warehouse-short-60s.mp4) · [180s](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.2.0/rosclaw-warehouse-promo-180s.mp4) · [510s](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.2.0/rosclaw-warehouse-tutorial-510s.mp4) · [Release](https://github.com/ros-claw/rosclaw-robotics-lab/releases/tag/warehouse-patrol-v0.2.0)

[English](README.md) · [中文教程](challenges/01-isaac-warehouse-patrol/docs/tutorial_zh.md) · [交付清单](deliverables/v02/STATUS.md)

| 冻结版本条件 | 全部尝试 | 进入 Native | PASS |
|---|---:|---:|---:|
| standard | 6 | 5 | 5 |
| reordered | 5 | 5 | 5 |
| unmapped-box | 5 | 5 | 4 |

Agent 观测 ROS 并逐站提议；Agentd/Operator/rosclawd 持有 Body 绑定的 SIM 执行权，Nav2 规划并控制运动。每站要求 Nav2 成功、独立物理误差 ≤0.4 米、稳定停留 ≥2 仿真秒、新鲜 LiDAR、完整接触观察与零非地面碰撞。完整有序收据及成功 Practice/Memory 验证之后，TaskKernel 才能结束。不使用 Agent 内 Runtime、原始轮速工具或一键整圈巡检工具。

箱体任务另验原始地图空闲证明、真实 LiDAR 命中、局部主 Costmap 占据与保守足迹净距。实际隔离 SIM 故障补测覆盖超时、断线、暂停后恢复停止，以及真实 Navigator ABORT 时真实 FollowPath 仍活动的取消与停止。所有失败保留在[故障索引](deliverables/v02/FAULT_TESTS.md)。暂停时不能凭旧物理数据确认停止，须另验恢复后的连续推进窗口。

实际运行 lab SHA：`09739cb34c8a1d37ad95cfa4f146ad1e47798ec3`；上游运行 SHA：`21838614bb14c39599b8acef731b2b64dad3b92a`。后续文档/测试提交与新上游改进 PR 分开记录，发行时核对运行文件哈希一致。见[完整指标](deliverables/v02/regression-metrics.json)、[性能口径](deliverables/v02/PERFORMANCE_PROTOCOL.md)、[Harness 审计](deliverables/v02/HARNESS_AUDIT.md)和[完整报告](deliverables/v02/IMPLEMENTATION_REPORT.zh.md)。

10 月 8 日历史五轮成功属于不同集成快照。三版视频都来自历史第五轮同一次任务，包含机器人前向、第三人称和近顶视同步画面；并非本轮十六次尝试的录像。60 秒版标记片段跳切，保留原时间和加速说明。编码 FPS 和重复最近真实帧不等于实采 FPS。[旧 v0.1](https://github.com/ros-claw/rosclaw-robotics-lab/releases/tag/warehouse-patrol-v0.1.0)与[冻结前失败](deliverables/v02/preliminary)独立保留。

```bash
cd challenges/01-isaac-warehouse-patrol
./scripts/setup.sh
python3 scripts/lab.py doctor
python3 scripts/lab.py start headless patrol
# 按教程启动观察者，再向真实 Native Agent 输入一句任务。
python3 scripts/lab.py stop
```

统一入口另提供 `task`、`results` 和十五次重置的 `regression SHORT_DIRECTORY`。启动不自动发导航目标；官方资产在线加载，使用者自行接受 NVIDIA 许可。Docker 基础镜像固定 manifest digest，apt 结果可能变化，每轮记录实际 image ID 与包版本。安装器、USD、缓存、认证、操作员密钥、私有模型目录和完整视频不进入 Git。

[独立复现交接包](deliverables/v02/reproduction/README.zh.md)已准备，用户暂未安排工程师和另一台机器，因此第三方实测为 **NOT RUN / 待安排**。离线重放只验证证据自洽。[Challenge 02](challenges/02-semantic-inspection/README.md)已从真实 USD/地图/位姿/Body 动态生成观察位置，完成开发集新目标导航及已知区域 LiDAR 验收；holdout 和公平对照尚未执行。运行对照与应用开发效率对照分开设计，不预设 ROSClaw 优于 Codex。当前 SIM 使用 SQLite、关闭可选 firewall 集成，未验收全部安全模块或 seekdb 性能。
