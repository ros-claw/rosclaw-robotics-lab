# Reproduce a real Native Agent warehouse patrol

Tested hardware: DGX Spark GB10, Ubuntu 24.04 aarch64, NVIDIA driver
580.159.03. Isaac Sim 6.1.0 runs on the host; ROS 2 Jazzy/Nav2 run in
an ARM64 container. No host ROS installation or driver replacement is needed.
A clean environment run by an independent engineer is still pending.

## 1. Prerequisites

Install Docker with permission for your user, Git, Node.js >=22, npm, Python 3,
and [uv](https://docs.astral.sh/uv/). Download NVIDIA's Isaac Sim 6.1.0
Linux aarch64 standalone package and accept NVIDIA's EULA yourself. Extract it
into `~/isaacsim`, run `post_install.sh`, then run the packaged compatibility
check. The tested ZIP MD5 was `02824f6b3c20d941ab81bf33d027d40b`.
Do not use the x86 package on Spark. Network access to official online assets,
GitHub, ROS package repositories and your configured model is required.

```bash
git clone https://github.com/ros-claw/rosclaw-robotics-lab.git
cd rosclaw-robotics-lab/challenges/01-isaac-warehouse-patrol
./scripts/setup.sh
./scripts/doctor.sh
```

Setup builds the public ARM64 Dockerfile, builds three NVIDIA ROS packages,
and installs a separate pinned ROSClaw checkout. Defaults are in
`config/runtime.env`; every path/image/domain can be overridden via environment
variables. Existing checkouts with a different revision are rejected rather than
modified. The pinned NVIDIA workspace commit is
`a9e8471ee901bc2332c1e4aca94ac580713ca3ab`; ROSClaw is
`21838614bb14c39599b8acef731b2b64dad3b92a`.
The Docker base digest is pinned; apt package versions can change. Consult the
per-run source/image manifest for the exact tested build.

Configure and authenticate a working model using the pinned ROSClaw checkout's
normal model workflow before acceptance. The runner copies your existing
`~/.rosclaw/agent` settings into a private isolated home. It does not switch
providers or export credentials. Recorded local runs used the existing
`openai-codex/gpt-6.1-sol` configuration.

## 2. Start the environment

```bash
ROSCLAW_CAMERA_VIEW=top ROSCLAW_CAPTURE_SECONDS=1400 ./scripts/demo.sh streaming patrol
```

Startup waits up to 20 minutes for online assets, shaders and independent PhysX,
then waits for ACTIVE Nav2 and NavigateToPose. It prints an absolute evidence
folder. Copy that exact path into `PHYSICS_DIR` below. There are no automatic
goals. Streaming can be replaced with `headless` or `gui`; viewport capture is
verified locally, external WebRTC client connectivity has not been validated.
Display-only camera choices are `official`, `top`, `overview`, `follow`, `top-close`, `robot`.
See [camera views](camera-views.md) for framing and native render resolution.
ROS domain 61 and loopback rosbridge port 19091 isolate this project. Choose a
free domain if another project already uses 61.

In a second terminal, leave the read-only probe and bridge running:

```bash
./scripts/start-agent-observers.sh
```

The session layer disables four nonessential Hawk ROS variants with cyclic
payload references. LiDAR remains active; source NVIDIA USD files are unchanged.
The patrol Nav2 configuration is separate from the official baseline. See
[the integration notes](patrol-integration.md) for each parameter change.

## 3. One task, real Agent decisions

Use a new run directory **outside any Git repository** on every reset. The chat
CLI discovers the enclosing Git workspace, so a directory inside this repository
can accidentally share a workspace. Do not reuse credentials/runtime directories
from another mission.

```bash
PHYSICS_DIR=/absolute/path/printed/by/demo
RUN_DIR="$HOME/sim/rosclaw-isaac-runs/$(date -u +%Y%m%dT%H%M%SZ)"
TASK='请先检查机器人状态，然后依次巡检仓库入口、货架区和仓库通道，遇到障碍物自主避让，完成后返回 Home，并保存巡检报告和记忆。每个位置停留检查实际位姿与激光数据。'
./scripts/run-native-acceptance.sh "$RUN_DIR" "$PHYSICS_DIR" "$TASK" entry shelf aisle home
```

This acceptance harness sends one user message to the real Native chat and
approves only its isolated, Body-bound SIM authorization cards. Site arguments
are an evaluator contract; they are not a route sent to Nav2. The Agent observes
ROS, proposes each `navigation.navigate_to_pose` call with one registered site ID,
then requests `patrol.verify_and_remember`. Physical actions pass through Agentd,
operatord, rosclawd and canonical receipts. MCP tools cannot directly move the
robot; the Agent does not construct a Runtime or register an executor.

For an order-variation test, reset the scene and change both the natural-language
task and independent expected order, for example `aisle entry shelf home`.
This still requires all three sites and a return to Home. Each site is measured
against the official map and composed USD inventory; arbitrary coordinates are
not accepted by the adapter.

## 4. Check evidence independently

```bash
python3 evaluator/run_report.py --directory "$RUN_DIR" --output reports/my-native-run.json
./scripts/stop.sh
```

PASS requires Nav2 status 4/error 0, physical error <=0.4 m, yaw <=0.35 rad,
>=2 simulated seconds of stable dwell, fresh valid LiDAR and complete contact
observation with zero non-floor pairs at every site. The canonical Body hash and
artifact SHA-256 must match. Existing Memory must contain a successful verified
experience, and TaskKernel must close only after the Memory receipt. Missing
observations cannot pass. Recovery is bounded to six reported Nav2 operations and
420 wall seconds per goal; timeout/cancel/error never becomes success.

Raw evidence, private model homes and PTY logs stay outside Git. Never upload an
entire run directory: it contains authentication and operator keys. Public report
summaries omit raw model thinking. Failure reports are retained alongside success.
`stop.sh` targets only labeled project containers and the recorded Isaac PID with
its process-birth identity. Let the Native harness finish cleanup before stopping
its simulator. Ctrl-C the harness to request cancellation and cleanup.

## 5. Optional baseline and fault experiments

After a fresh reset, `./scripts/smoke-official.sh` runs NVIDIA's three-goal
baseline. It is not Agent acceptance. Original defaults missed the physical
0.4 m bound once; calibrated baseline errors were 0.146/0.174/0.079 m.

`ROSCLAW_CONTACT_TEST=1` creates an isolated collision positive control and must
never be enabled for acceptance. `ROSCLAW_OBSTACLE_TEST=1` adds a session-layer
unmapped static box for a separate obstacle test; it is off by default. Run the
read-only `ros2/obstacle_witness.py` in the ROS container to record actual pointcloud,
costmap and plan evidence. Do not call an untested obstacle case successful.

Video generation uses actual timestamped viewport captures, independent PhysX
and recorded PTY output. The 80-second promotional mode explicitly labels time
compression. Workflow mode preserves the full recorded wall interval at 2 fps;
it is a frame-based reconstruction, not a continuous screen recorder. A standalone
renderer is provided in `visualization/render_video.py`; it needs Pillow, PyYAML,
pyte, Noto CJK/DejaVu fonts, the official map PNG/YAML and an ffmpeg executable.

Download the public evidence ZIP from Releases and check it without Isaac or a model:

```bash
python3 evaluator/replay_archive.py native-acceptance-2-evidence.zip
```

This checks the recorded receipt/hash/physical evidence only. It is not a live
clean-machine reproduction. Select a fixed display view at launch; changing the
camera projection during a live session was observed to stall Isaac and has been
removed. The tested top view stays below the warehouse roof.

### Reproduce the independently gated unmapped-box case

Stop the previous session, then use `ROSCLAW_OBSTACLE_TEST=1` when starting a
fresh environment. Start the bridge/probe in a second terminal as above. In a
third terminal, record actual sensor/costmap/path evidence before sending the task:

```bash
EVIDENCE_NAME=$(basename "$PHYSICS_DIR")
ROSCLAW_CONTAINER_NAME=rosclaw-warehouse-path-witness ./scripts/ros-container.sh \
  python3 /lab/ros2/obstacle_witness.py \
  --output "/lab/reports/runs/$EVIDENCE_NAME/path-witness.jsonl" \
  --ros-args -p use_sim_time:=true
```

Then use a new private run directory and a task specifying the box and the order
Aisle → Entry → Shelf → Home:

```bash
ROSCLAW_REQUIRE_OBSTACLE_EVIDENCE=1 ./scripts/run-native-acceptance.sh \
  "$RUN_DIR" "$PHYSICS_DIR" "$TASK" aisle entry shelf home
```

The extra flag is an acceptance contract, not an Agent navigation program. It
requires actual LiDAR hits, local master-costmap occupied cells, conservative
footprint clearance and zero contacts before Memory success. Static-map free-cell
proof is bound to the actual official map file hash. Merely reaching the site is
insufficient. The first insufficient case and the corrected full case are both
publicly reported. Package only whitelist evidence with
`visualization/package_evidence.py`; its offline replay includes this extra gate.
