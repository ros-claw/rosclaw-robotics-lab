# ROSClaw Robotics Lab

**One natural-language task → a physically verified warehouse patrol.**

A real ROSClaw Native Agent controls Nova Carter in NVIDIA's official Isaac Sim
6.1 warehouse through ROS 2 Jazzy/Nav2 on DGX Spark. **Four complete reset missions
passed**, including changed natural-language order and an unmapped static obstacle.
These are SIM results; real-hardware execution is not verified.

[![Actual Native Agent demo](challenges/01-isaac-warehouse-patrol/reports/native-demo-poster.png)](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.1.0/rosclaw-warehouse-promo.mp4)

[Watch the 80-second demo](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.1.0/rosclaw-warehouse-promo.mp4)
· [Full Native workflow](https://github.com/ros-claw/rosclaw-robotics-lab/releases/download/warehouse-patrol-v0.1.0/rosclaw-warehouse-workflow.mp4)
· [Evidence archives](https://github.com/ros-claw/rosclaw-robotics-lab/releases/tag/warehouse-patrol-v0.1.0)
· [中文](README.zh.md)

The Agent observes ROS, proposes one registered site at a time, monitors results,
returns Home and saves a verified experience in existing Practice/Memory. Only
rosclawd owns physical execution. No fake model, Agent-side Runtime or one-shot
patrol executor is used. Each visit requires Nav2 success, <=0.4 m physical error,
stable dwell >=2 simulated seconds, fresh LiDAR and zero non-floor contacts.
TaskKernel closes only after the final Memory receipt.

| Reset | Agent-selected order | Independent result |
|---|---|---|
| Native 1 | Entry → Shelf → Aisle → Home | PASS |
| Native 2 | Entry → Shelf → Aisle → Home | PASS |
| Native 3 | Aisle → Entry → Shelf → Home | PASS |
| Native 4, public Docker build + unmapped box | Aisle → Entry → Shelf → Home | PASS; actual errors 0.095 / 0.094 / 0.132 / 0.090 m |

Maximum error across these 16 visits: **0.151 m** rounded upward. The obstacle
case recorded 6,368 actual LiDAR box hits, 367 local master-costmap occupied cells,
>=0.35 m conservative footprint clearance and zero non-floor contacts. Actual
movement followed by timeout cancellation and transport-loss DDS cancellation
also passed separate negative tests.

```bash
cd challenges/01-isaac-warehouse-patrol
./scripts/setup.sh
./scripts/doctor.sh
ROSCLAW_CAMERA_VIEW=top ROSCLAW_CAPTURE_SECONDS=1400 ./scripts/demo.sh streaming patrol
# In a second terminal:
./scripts/start-agent-observers.sh
# Send one task with the real Native runner as described in the tutorial.
```

Read [the English tutorial](challenges/01-isaac-warehouse-patrol/docs/tutorial_en.md),
[中文教程](challenges/01-isaac-warehouse-patrol/docs/tutorial_zh.md),
[execution architecture](challenges/01-isaac-warehouse-patrol/docs/architecture.md),
[retained failures](challenges/01-isaac-warehouse-patrol/docs/failures.md), and
[implementation report](FINAL_IMPLEMENTATION_REPORT.md).
The [acceptance summary](challenges/01-isaac-warehouse-patrol/reports/acceptance-summary.json)
records actual source snapshots, Body hashes, image IDs, SDK model turns and metrics.

Startup sends no goals. `smoke-official.sh` is the separate NVIDIA baseline.
Videos use actual timestamped viewport captures, recorded visible terminal output
and independent PhysX. Promo time compression is labeled; full workflow preserves
wall time at 2 fps. They are frame-based reconstructions, not continuous screen
recordings. Private model thinking, authentication and operator keys are excluded.

Passing resets used documented source revisions during integration. Failed runs
remain available; these successes do not establish a statistical success rate.
The public ARM64 Dockerfile and its runtime were tested locally. An
[independent engineer's clean-machine reproduction](challenges/01-isaac-warehouse-patrol/docs/reproduction-checklist.md)
is still pending. External WebRTC clients, China asset loading and dynamic
pedestrians are not validated.

Sources: [ROSClaw](https://github.com/ros-claw/rosclaw),
[NVIDIA workspace](https://github.com/isaac-sim/IsaacSim-ros_workspaces/tree/IsaacSim-6.1.0).
No NVIDIA USD, installer, cache, private model home or full video is committed.
