# ROSClaw Robotics Lab

Goal: one natural-language ROSClaw session, one independently verified warehouse patrol.

**Current delivery: the Isaac Sim / Nova Carter / Nav2 simulation baseline. Agent patrol is not implemented.**

![Actual warehouse viewport](challenges/01-isaac-warehouse-patrol/reports/warehouse-baseline.png)

Verified on DGX Spark with Isaac Sim 6.1.0 and ROS 2 Jazzy: installation, compatibility, online scene rendering, independent PhysX state collection and three Nav2 goals. Calibrated physical errors: **0.146 / 0.174 / 0.079 m**. These are navigation baseline results, not Agent acceptance.

```bash
cd ~/sim/rosclaw-robotics-lab/challenges/01-isaac-warehouse-patrol
./scripts/doctor.sh
./scripts/demo.sh streaming
./scripts/smoke-official.sh  # optional official baseline only
./scripts/stop.sh
```

Startup waits for independent Physics and an ACTIVE Nav2 action server. It sends no automatic goals. This installation currently depends on a prepared local ARM64 Docker image; clean-machine reproduction remains pending.

See [English operations](challenges/01-isaac-warehouse-patrol/docs/tutorial_en.md), [中文说明](README.zh.md), and [implementation status](FINAL_IMPLEMENTATION_REPORT.md).

Sources: [ROSClaw](https://github.com/ros-claw/rosclaw), [NVIDIA workspace](https://github.com/isaac-sim/IsaacSim-ros_workspaces/tree/IsaacSim-6.1.0). No NVIDIA USD, installer, cache or full video is committed.
