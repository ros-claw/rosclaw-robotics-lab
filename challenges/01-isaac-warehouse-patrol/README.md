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
