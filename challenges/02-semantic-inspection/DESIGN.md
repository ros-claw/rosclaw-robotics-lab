# Challenge 02 execution and comparison protocol

## Task and evidence

Example instruction: “去仓库北侧货架找到合适的观察位置，检查后经过中间通道返回起点。” The task planner selects semantic regions/objects from a read-only catalog. The catalog identifies USD-provided semantics explicitly; no sensor-discovery claim is made. Candidate coordinates are generated from source bounds and the current map, not from the four registered patrol stations.

Stages: intent → semantic object selection → candidate generation → static clearance/reachability → fresh Nav2 path/costmap validation → Body/action-intent validation → operator SIM authorization → daemon execution → independent arrival, facing and inspection evidence → final ordered mission receipt → existing Practice/Memory.

## Runtime integration boundary

Reuse the existing ROS Expert Harness snapshot, proposal/Body validator, rosclawd Session/Lease/Permit/action ledger and single-goal Nav2 executor. Do not introduce a second runtime, memory store or direct motion tool. Add a read-only candidate capability returning immutable proposal IDs. A daemon-owned capability resolves an ID, rejects stale/mutated sources and verifies the exact Body snapshot; the Agent cannot replace a proposal with arbitrary x/y.

The current patrol Body authorizes a registered-site contract. New generated targets must **not** be inserted into that contract after its snapshot is bound. Compile and review a separate SIM Body action contract defining allowed observation regions and proposal validation, then bind that immutable contract before task input. The daemon must enforce it. A read-only proposal alone grants no motion authority.

Nav2 ComputePathToPose must succeed for the fresh current pose/target. Independently sample the path with the Body footprint at sufficient spatial/angular resolution against the current costmap, including unknown cells and outside-map bounds. Compare map/TF frames and timestamps. Revalidate on source changes; expire proposals within a measured wall/SIM freshness budget. Restrict recovery attempts/time; retain every failed/rejected target.

“Inspection” needs a concrete observation contract: at minimum target-facing pose plus actual target-visible sensor evidence with known timestamp/calibration. Merely arriving or rendering a display camera does not verify inspection. First milestone may honestly be named *semantic observation-point navigation*, followed by sensor inspection acceptance.

## Held-out targets

The prototype groups parent/child RackShelf prims into the outermost physical shelf Xform, then hash-splits entity paths before testing candidate quality. The current actual inventory yields 8 development entities and 5 holdout entities. Only one development entity has been evaluated for candidate generation; no holdout outcomes have been accessed. A preliminary prim-level split was discarded to eliminate parent/child leakage. Freeze the entity manifest and SHA outside Agent prompts before evaluation; reveal targets only after code, thresholds and prompts are frozen. Report held-out failures and unsafe rejections as well as successful motion. The present preparation is not a completed held-out experiment.

## Fair task-runtime comparison

Use the same model/provider/version/account settings, token limits, Nav2 interface, candidate source data, robot Body, map, initial poses, obstacles, recovery/time budget and independent evaluator. Both ROSClaw and generic Agent receive equal raw observations and equivalent constrained task tools. Neither receives a prebuilt task-sequence tool. The baseline also uses the same daemon physical boundary; otherwise safety and tool design are confounded with model capability.

Preregister randomized paired orders and environment seeds (start with 30 pairs per task family after pilot sizing). Reset environment and memory for every pair; freeze prompts before evaluation. Record one-task-input acceptance, proposal rejection, physical task success, collisions, wrong semantic target, latency, tokens, tool calls, interventions and cost. Publish all runs, paired differences and uncertainty. Failed/time-limited runs remain in the denominator; success-only latency is secondary. No winner is assumed.

## Fair application-development comparison

This is a separate study, starting when a developer/Agent receives an existing ROS robot environment and the same task brief. It includes diagnosis, capability selection, integration and first independent physical acceptance. Record wall/active engineering time, human interventions with reasons, files/lines changed, configuration changes, failed attempts and repairs. Infrastructure installation/caching is recorded separately. Assign equivalent fresh environments and counterbalance experienced developers to reduce learning effects. Keep developer identities independent of task implementation where possible.

The runtime experiment cannot establish development-time benefit. Existing warehouse engineering was done with Codex and runtime orchestration by ROSClaw; there is no measured ROSClaw-versus-Codex advantage yet.

## Acceptance status

- Static candidate generation on actual map/USD/PhysX/Body inputs: demonstrated on one development target; read-only proposal only.
- Nine deterministic gate regressions: pass (unknown occupancy, stale/paused state, contacts, Body binding, margins, split restriction, actual geometry/hash sensitivity).
- Fresh Nav2 path and daemon dynamic-target execution: pending.
- Sensor inspection, held-out generalization, paired Agent comparison and development-time study: pending.
