# Implementation status — 2026-10-08

The initial warehouse simulation/navigation milestone is implemented and measured. The complete natural-language ROSClaw patrol described in the proposal is not yet delivered.

## Verified on this DGX Spark

- Ubuntu 24.04.4 aarch64, GB10 driver 580.159.03; host driver and ROS were not replaced.
- NVIDIA ZIP MD5 matched 02824f6b3c20d941ab81bf33d027d40b. Isaac installed at /home/nvidia/isaacsim; EULA acknowledged, post_install succeeded, compatibility PASSED and warmup rendered NEW_FRAME.
- NVIDIA ROS workspace pinned to a9e8471ee901bc2332c1e4aca94ac580713ca3ab (IsaacSim-6.1.0), recursive submodules; three required official packages built in ARM64 Jazzy Docker. Nav2 and pointcloud_to_laserscan verified in the prepared image.
- Official online warehouse scene contains one Nova Carter; actual viewport screenshot and independent PhysX chassis state recorded. Original USD files are unchanged.
- Domain 61, Fast DDS with host network/IPC Docker receives progressing clock, odometry, LiDAR points, scan and map. map→odom→base_link TF is available. Nav2 reaches ACTIVE with a ready NavigateToPose server.
- Official three-goal baseline: all three bt_navigator logs report Goal succeeded. Physical errors 0.107 / 0.297 / 0.543 m. Third goal FAILS 0.4 m accuracy.
- Separate reset and calibration candidate (AMCL alpha1–5 0.001, Nav2 XY tolerance 0.10 m): physical errors 0.146 / 0.174 / 0.079 m, all below 0.4 m. Baseline accuracy PASS only; this is not Agent mission acceptance.
- Independent baseline scorer rejects missing/stale physics, excessive physical error and error_code=0 without Nav2 success evidence; four unit checks passed. Python static checks and shell syntax checks passed.
- Fresh ROSClaw upstream pinned at 21838614bb14c39599b8acef731b2b64dad3b92a, isolated installation and Native Agent build completed; ROS native probe, graph discovery and deep system inspection actually ran. Existing G12 experimental runtime was not replaced.
- Local timestamped viewport frame-sequence recording: challenges/01-isaac-warehouse-patrol/reports/official-baseline.mp4. It is actual baseline imagery, not continuous screen capture or an Agent promotional video.
- One-button startup successfully reached independent Physics plus Nav2 ACTIVE/READY without any automatic goal sender, both before and after the camera variant workaround. Local Chinese/English operating guides are included.

## Limitations and remaining proposal work

- Hawk camera payload cycles were found in NVIDIA assets; the session-layer Disabled ROS variant workaround was verified on a fresh startup. Composition errors=0 and unloaded payloads=0. Camera render products remain disabled for navigation. Full texture validation, China network loading and external WebRTC client viewing remain unverified.
- Collision events are recorded but ground classification has not been accepted; collision status remains UNKNOWN. No claim of zero validated obstacle collisions.
- Semantic Entry/Shelf/Aisle/Home sites, overlays and formal patrol task contracts are pending. Baseline goals are measured map pixels, not semantic inspection locations.
- Native Agent patrol, a minimal Isaac SIMULATION rosclawd executor, Body binding, real task decisions, receipts, Practice/Memory, reordered instruction, bounded recovery and three independent Agent runs are pending.
- Promotional/tutorial videos, portable clean-machine Docker installation, independent engineer reproduction and GitHub publication are pending.
- A stale saved Harness snapshot correctly diagnoses BLOCKED; saved historical evidence is not live execution authorization.

## Evidence

- reports/environment.json and docs/environment-audit.md: machine, dependencies, failures and repairs.
- reports/calibrated-baseline-summary.json: measured errors and hashes of frozen raw logs/physics.
- reports/warehouse-baseline.png: actual renderer output.
- reports/verified-runtime-audit.json: after-camera-fix clock, scan, LiDAR, odometry and map reception plus TF.
- reports/verified-scene-audit.json: final composed scene audit, including the four disabled Hawk ROS variants, zero composition errors and 106 loaded layers.
- reports/native-probe.json, ros-graph.json, system-model.json and runtime-audit.json: historical read-only ROS evidence.
- reports/official-calibration/: all calibration data retained, including an initial action-server readiness timeout. It was not discarded as a successful run.
- reports/runs/: independent startup/reset evidence, with fresh scene-audit.json and physics trajectory.

Large assets, raw frame directories and videos stay local and are excluded from Git. No fabricated coordinates, receipts or task-success claims are included.
