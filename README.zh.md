# ROSClaw Robotics Lab

目标：通过 ROSClaw 的真实自然语言会话完成仓库巡检，并独立验证物理结果。

**当前交付：Isaac Sim + Nova Carter + Nav2 仿真基线。ROSClaw 自主巡检尚未实现。**

![实测仓库画面](challenges/01-isaac-warehouse-patrol/reports/warehouse-baseline.png)

DGX Spark / Isaac Sim 6.1.0 / ROS 2 Jazzy。安装与兼容性检查通过；官方在线场景已实际渲染、运行物理并完成三个导航目标。校准配置的独立物理误差为 **0.146 / 0.174 / 0.079 m**。这只证明仿真导航基线，不是 Agent 任务验收。

本机最短启动步骤：

```bash
cd ~/sim/rosclaw-robotics-lab/challenges/01-isaac-warehouse-patrol
./scripts/doctor.sh
./scripts/demo.sh streaming
# 单独执行官方三目标环境测试（不会计入 Agent 演示）
./scripts/smoke-official.sh
# 停止本项目启动的进程
./scripts/stop.sh
```

`demo.sh` 等待独立物理采样及 Nav2 ACTIVE/Action READY；不会自动发送目标。当前依赖本机已准备的 ARM64 Docker 镜像，尚未完成干净机器复现验收。

- [本机中文操作教程](challenges/01-isaac-warehouse-patrol/docs/tutorial_zh.md)
- [环境与修复记录](challenges/01-isaac-warehouse-patrol/docs/environment-audit.md)
- [完整实施状态与未完成项](FINAL_IMPLEMENTATION_REPORT.md)
- [ROSClaw](https://github.com/ros-claw/rosclaw)
- [NVIDIA 官方固定工作空间](https://github.com/isaac-sim/IsaacSim-ros_workspaces/tree/IsaacSim-6.1.0)

原始 USD、安装包、缓存与视频不提交 Git。真实录像保存在本机 reports/official-baseline.mp4；没有宣传片、Agent 成功声明或伪造的 ExecutionReceipt。
