# Reproduce warehouse loading-area inspection

Use the existing NVIDIA warehouse, Nova Carter, ROS 2 Jazzy/Nav2 and ROSClaw Native → Agentd/Operator → rosclawd chain. This challenge adds bounded observation and measured passage evidence. It does not add another controller or agent runtime.

## Challenges and gates

| Case | Challenge | Required behavior |
|---|---|---|
| A | Original warehouse; choose the forklift near shelves geometrically | Inspect from safe proposals, report measured result, return to actual start, verify Memory |
| B | A colliding temporary box absent from the static map | Detect measured extra occupancy and separately evaluate passage clearance |
| C | Start nearer the other forklift; actual forklift occludes the first view | Select the shelf-adjacent forklift, make a second safe observation with ≥3 new measured cells, at most two views |
| D | Immutable Body observation bounds exclude all generated viewpoints | No unauthorized inspection motion; explicit UNKNOWN, verified return and Memory |

The original area can be OBSTRUCTED. A nearby forklift can block the required full-footprint entrance even when no additional box exists. Task PASS and inspection CLEAR are different concepts. D tests a constrained Body authorization refusal, not every narrow-space or unreachable-world condition.

## Setup and run

Follow [Challenge 01 setup](../../01-isaac-warehouse-patrol/README.md) for the licensed Isaac installation, ROS container and configured Native model. The tested actual merged ROSClaw source/build is `9862058c445c88156b9080832b5dd803c75279ae`; frozen lab implementation is `d1a1db271ae0030ea47df415d890751dcfd205fb`. Later delivery commits may add documentation without rewriting these evidence pins.

Use the `loading-area-inspection-v0.4.0` delivery tag for documentation and fixtures, and check executable hashes against `deliverables/v04/runtime-binding.json`. For an exact historical d1a1db2 checkout, first save release fixtures outside the repository and pass their external paths; that historical commit predates delivery documentation. Never rewrite the measured SHA.

```bash
export ROSCLAW_SOURCE=/path/to/exact/merged/rosclaw
export PYTHONPATH="$ROSCLAW_SOURCE/src${PYTHONPATH:+:$PYTHONPATH}"
export ISAAC_ROS_WS="$HOME/IsaacSim-ros_workspaces/jazzy_ws"
export ROS_CONTAINER_IMAGE=rosclaw/warehouse-jazzy:6.1.0-public
npm ci --prefix "$ROSCLAW_SOURCE/packages/rosclaw-agent"
npm run build --prefix "$ROSCLAW_SOURCE/packages/rosclaw-agent"
cat "$ROSCLAW_SOURCE/packages/rosclaw-agent/dist/build-stamp.json"

# From the lab repository root; use a fresh, short output path every time.
"$ROSCLAW_SOURCE/.venv/bin/python" challenges/02-semantic-inspection/run_loading.py \
  --output "$HOME/sim/loading-a1" --isolated-simulation \
  --fixture deliverables/v04/fixtures/a1.json

"$ROSCLAW_SOURCE/.venv/bin/python" challenges/02-semantic-inspection/run_loading.py \
  --output "$HOME/sim/loading-b1" --isolated-simulation \
  --fixture deliverables/v04/fixtures/b1.json

"$ROSCLAW_SOURCE/.venv/bin/python" challenges/02-semantic-inspection/run_loading.py \
  --output "$HOME/sim/loading-c1" --isolated-simulation \
  --fixture deliverables/v04/fixtures/c1.json --record-multiview

"$ROSCLAW_SOURCE/.venv/bin/python" challenges/02-semantic-inspection/run_loading.py \
  --output "$HOME/sim/loading-d1" --isolated-simulation \
  --fixture deliverables/v04/fixtures/d1.json \
  --authorized-observation-bounds deliverables/v04/fixtures/d1-bounds.json
```

Do not overlap project sessions. Existing scripts use ROS Domain 61, localhost and project-owned container/process identities. Never copy public evidence into a private Native HOME or commit model authentication/operator keys.

Each invocation resets the simulation, compiles a separate immutable SIM Body, sends the same Chinese business task exactly once, runs Native, evaluates receipts and cleans owned resources. The task asks to inspect the loading area and forklift near shelves, choose safe viewpoints, supplement insufficient observations, return to the actual start and report findings and uncertainty. There is no user-supplied pose, USD path or test label. Operator SIM decisions are action authorizations rather than extra task inputs. No REAL authority is granted.

Reset fixture labels/seeds and temporary-box truth are evaluation-only. Known facility geometry is captured before applying the hidden fixture. Robot observations use actual PointCloud2, not PhysX truth. Actual tool-call audits accompany the evidence; this same-UID engineering environment is not an OS sandbox against a malicious agent reading arbitrary files.

## Evidence and limits

The actual odometry topic is `/chassis/odom`, not `/odom`. Raw `/front_3d_lidar/lidar_points` buffers retain fields, strides, timestamps, SHA256 and measurement-time TF. Offline evaluation independently decodes and transforms the original bytes.

Ground uses z=0±0.08m; declared obstacle evidence is 0.10–0.80m above ground. A 0.2m cell needs finite measured rays in all three height bands. Predicted visibility only ranks proposals; missing returns and unknown cells provide no free-space credit. Finite sampling cannot rule out all thin objects or obstacles below 0.10m.

The actual configured asymmetric footprint has a 0.65647m enclosing radius. Inspection uses 0.75m plus a half-cell diagonal guard, conservatively rounded to a five-cell square stencil. A measured halo outside the ROI supplies boundary footprint evidence. Nav2 path computation and full swept-footprint gates remain separate and mandatory.

CLEAR needs ≥70% measured ROI coverage and a verified free passage; OBSTRUCTED needs measured clearance blockage across the passage; UNKNOWN preserves insufficient or invalid evidence. Coverage below 95% or UNKNOWN can motivate a second viewpoint. The actual gain must be measured, with at most two inspection views. Correct OBSTRUCTED or honest UNKNOWN can complete the task successfully.

Inspect `result.json`, `acceptance.json`, canonical receipts, `native/actions/`, `semantic-evidence/*cloud.json`, source freezes and the independent physics directory. `loading-physx-truth.json` is evaluation-only. Recording coverage is tested separately in `recording-acceptance.json`.

The 90-second video uses synchronized actual third-person, robot-facing and close top views, plus actual measured cloud/cell evidence. RGB frames are presentation cameras, not a visual inference channel. Capture rate, encoding rate, wall acceleration and SIM/wall ratio are recorded separately.

Release ZIPs contain explicit evidence whitelists and hashes, never model thinking, private sessions, HOME or credentials. The delivered replay tool verifies archive manifests and runs the matching frozen evaluators. Offline consistency is not independent reproduction by a new engineer on a second machine.

Legacy `run.py --require-target-lidar` remains supported. Historical v02/v03 evidence and denominators remain separate. The new small frozen suite is engineering regression, not production reliability, arbitrary-scene generalization, RGB goods inspection or a fair comparison against Codex. See the Chinese tutorial for the complete task and detailed operational notes.

## Re-render the public camera sources

Extract the formal evidence ZIP into `EVIDENCE` and all three `loading-camera-*.zip` archives into one new `FRAMES` directory. It must contain the measured `timestamps.jsonl` and corresponding PNG paths. Run:

```bash
"$ROSCLAW_SOURCE/.venv/bin/python" deliverables/v04/rerender_video.py \
  --attempt EVIDENCE/l04qc07 --frames-directory FRAMES \
  --output /tmp/loading-video-replay.mp4 --ffmpeg /path/to/ffmpeg
```

This wrapper relocates media lookup paths only in a temporary workspace and preserves original task evidence. The delivered renderer still requires physical and recording PASS. The source ZIPs contain the exact selected frames and their hashes. Different encoding environments can change the output byte hash; re-rendering is not another physical trial.
