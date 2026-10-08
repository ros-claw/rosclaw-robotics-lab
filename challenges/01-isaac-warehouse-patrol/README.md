# 01 — Autonomous warehouse patrol

Four real ROSClaw Native Agent reset missions passed: all three inspection sites
and Home, changed task order, and a separate full mission with an unmapped box.
The Agent chooses individual Body-bound Nav2 calls from one task. Independent
PhysX/contact/LiDAR evidence and canonical receipts gate existing Practice/Memory
and final TaskKernel completion. Failure records are retained.

See [English tutorial](docs/tutorial_en.md), [中文教程](docs/tutorial_zh.md),
[integration design](docs/patrol-integration.md), [architecture](docs/architecture.md),
[acceptance summary](reports/acceptance-summary.json), and
[unmapped-obstacle Native acceptance](reports/native-acceptance-4.json).

`demo.sh` starts only the environment. `smoke-official.sh` is the separate NVIDIA
baseline. Download actual videos and whitelist evidence from the root README's
release links. Independent clean-machine reproduction remains pending.
