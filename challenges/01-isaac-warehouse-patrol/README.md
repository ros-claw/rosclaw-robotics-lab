# Challenge 01 — Isaac Sim Warehouse Patrol

One natural-language task, a real ROSClaw Native Agent, Body-bound rosclawd SIM execution, Nav2 and independently verified PhysX/LiDAR/contact receipts.

v0.2 frozen-source regression: **14/16 attempts accepted**. Every condition has at least five independent Native trials; one pre-Native startup failure and one provider-aborted partial task remain in the denominator. Actual condition counts and known limitations are in the [delivery status](../../deliverables/v02/STATUS.md). The extension goal of five accepted missions per condition is `False`. Historical fifth-mission videos and v0.1 results are separate.

[中文 tutorial](docs/tutorial_zh.md) · [English tutorial](docs/tutorial_en.md) · [Full current report](../../deliverables/v02/IMPLEMENTATION_REPORT.zh.md) · [Historical five-mission report](../../deliverables/v02/HISTORICAL_IMPLEMENTATION_20261008.zh.md)

```bash
./scripts/setup.sh
python3 scripts/lab.py doctor
python3 scripts/lab.py start headless patrol
# In another terminal: ./scripts/start-agent-observers.sh
# Then use the exact Native task command and evaluator from the tutorial.
python3 scripts/lab.py stop
```

Startup sends no goals. The unified `task`, `results` and independent-reset `regression SHORT_DIRECTORY` commands reuse the existing scripts. Keep results outside Git with short socket paths. The 15-trial regression command retains failures; it does not automatically rerun until success.

Use official online NVIDIA assets and your own model authentication/license acceptance. No installer/USD/cache, private Native home or credentials belong in Git or public evidence. [Independent reproduction](../../deliverables/v02/reproduction/README.md) is NOT RUN; the handoff is ready. [Actual fault CLI](../../deliverables/v02/FAULT_TESTS.md), [performance CLI](../../deliverables/v02/PERFORMANCE_PROTOCOL.md) and the offline verifier provide separate, explicit scopes.

## Task variants and acceptance

| Variant | One-input task | Challenge and what it tests |
|---|---|---|
| Standard | 入口 → 货架区 → 通道 → Home | Agent chooses individual registered goals and verifies the full sequence |
| Reordered | 通道 → 入口 → 货架区 → Home | Order comes from the instruction; the executor contains no patrol sequence |
| Unmapped box | Reordered route, with a physical box absent from the original map | Actual LiDAR/master-costmap evidence and conservative clearance, plus zero non-floor contact |

任务难点：标准巡检要求分步动作后继续任务并完成最终闭环；改序巡检要求实际顺序服从新指令；未映射箱体要求将实际 LiDAR、局部代价地图与物理净距联系起来，不能凭静态地图或导航返回成功就判定避障通过。

Each visit requires Nav2 success, ≤0.4 m independent position error, ≤0.35 rad yaw error, ≥2 simulation seconds stable dwell, fresh LiDAR and complete collision observation. Final ordered canonical receipts, verified Practice/Memory and subsequent TaskKernel success are required. Startup failures, model errors and interventions remain in the denominator.

## Watch the task

[![Three synchronized views: robot-forward, third-person and close top](reports/v02-demo-poster.png)](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.2.0/rosclaw-warehouse-promo-180s.mp4)

[60-second excerpts](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.2.0/rosclaw-warehouse-short-60s.mp4) · [180-second promo](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.2.0/rosclaw-warehouse-promo-180s.mp4) · [510-second tutorial](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.2.0/rosclaw-warehouse-tutorial-510s.mp4)

All three videos depict the same historical fifth mission, with source time/speed and excerpt labels. They are separate from the v0.2 sixteen-attempt regression and do not demonstrate Challenge 02.
