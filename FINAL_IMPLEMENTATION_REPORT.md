# Isaac warehouse patrol implementation report

Updated 2026-10-08. **Four independent full reset missions passed**, including
changed natural-language order and a four-site unmapped-obstacle run using the
public ARM64 Docker build. Source, independent evaluation, bilingual tutorials,
actual-frame videos and whitelist evidence archives are implemented. Independent
engineer reproduction on a clean machine remains pending.

## Actual execution

The existing configured model was `openai-codex/gpt-6.1-sol`, with pinned ROSClaw
`21838614bb14c39599b8acef731b2b64dad3b92a`. Native Agent, Agentd, TaskKernel,
operatord and rosclawd were real processes. The Agent observed ROS and proposed
individual sites. Only the daemon registered the single-site Nav2 executor;
MCP declarations cannot execute motion and no one-shot patrol tool exists.
Canonical SIM receipts bind the compiled measured eURDF/Body and visit artifact
hashes. No upstream ROSClaw checkout modifications or experimental global ROSClaw
installation were used for these missions.

The independent verifier requires Nav2 status 4/error 0, <=0.4 m physical error,
yaw <=0.35 rad, >=2 SIM seconds stable dwell, fresh valid LiDAR, complete floor
classification and zero non-floor contact pairs. A per-reset observer UUID prevents
cross-reset continuation. Existing RosPracticeAdapter/Memory must persist success,
and TaskKernel must close after its final receipt. Missing/non-finite evidence,
cancel, collision or early completion cannot pass.

## Measured results

| Full reset | Order | Result |
|---|---|---|
| isaac-a1 | Entry, Shelf, Aisle, Home | PASS |
| a3 | Entry, Shelf, Aisle, Home | PASS |
| a4b | Aisle, Entry, Shelf, Home | PASS |
| unmapped-box-final | Aisle, Entry, Shelf, Home | PASS with independent obstacle gate |

Maximum error across 16 visits was 0.15019 m. The fourth reset used image
`sha256:43cca6319ba787aed099db6cceb4ca22de4a24ab0d24b0f9a8b1ef14399ec232`;
errors were 0.0952/0.0941/0.1319/0.0902 m, all with zero recovery. Environment
startup took 54.19 wall seconds in that warm-cache run. Its observed SIM/wall ratio
was 0.318; promotional speedup is editing, not improved simulation performance.

The unmapped 1 m box occupies 441 all-free pixels of the unchanged official map.
The actual witness recorded 6,368 LiDAR hits, 367 local master-costmap occupied
cells and minimum conservative footprint clearance 0.35888 m. The registered
footprint's enclosing circle is used for the clearance bound. Zero non-floor
contacts and all four arrivals were independently verified. The final Memory
executor additionally gates obstacle evidence before publishing success.

Separate real adapter negative tests verified movement followed by deadline
cancellation (Nav2 CANCELED) and lost rosbridge transports followed by direct DDS
CancelGoal acknowledgement with the actual goal UUID and measured stop. These
fault tests are not Native mission success. Recovery-limit cancellation, preflight
rejections, premature TaskKernel closure, renderer/view failures and an archive
snapshot race remain retained; see [failures](challenges/01-isaac-warehouse-patrol/docs/failures.md).

## Reproduction and evidence

The NVIDIA workspace is pinned to IsaacSim-6.1.0 commit
`a9e8471ee901bc2332c1e4aca94ac580713ca3ab`. Its three required packages were built.
The public ARM64 Dockerfile pins the ROS base digest and was built through a
project-specific proxy-enabled BuildKit container. No host driver, host ROS or
Docker daemon proxy configuration was changed. Apt versions remain time-dependent
and exact tested package/image versions are recorded.

The four passing resets used documented integration revisions and Body hashes.
Each private run contains frozen executable source snapshots and a source manifest;
public evidence archives reproduce those snapshots. They do not claim all trials
used the eventual Git commit. Source/hash, canonical receipts, physical arrival,
Memory/TaskKernel and obstacle replay pass offline for all four archives. The
packager reads each file once and hashes the exact archived bytes; incomplete live
JSONL tails are excluded. Hash checking is an audit mechanism, not independent
proof that a new machine can run the task.

Sixteen pure gate unit tests, source checks and shell syntax checks passed. CI
runs those checks without GPU, licensed Isaac assets or authenticated model calls.
SDK sessions provide actual model turn counts; the pinned provider path had no
TaskKernel metering rows. Generic existing ROS Memory duration fields are not
used as physical timing metrics.

The 80-second promotional video and full Native workflow contain actual viewport
frames, timestamped visible terminal replay and independent PhysX overlays. They
are frame-based reconstructions; full mode preserves the complete recorded Native
wall interval at 2 fps. Installation/startup commands are covered by written
bilingual tutorials, not presented as recorded terminal steps in that video.
Private thinking is hidden in the actual TUI and never exported; authentication,
operator keys and entire private runtime homes are excluded. Videos and evidence
ZIPs are release assets rather than Git blobs.

## Pending external acceptance

An engineer uninvolved in development must still reproduce the tutorial on a
clean environment. This cannot be replaced by local Docker tests, CI or replaying
our archives. The [reproduction record](challenges/01-isaac-warehouse-patrol/docs/reproduction-checklist.md)
is explicitly pending until an engineer and machine are assigned.

External WebRTC client connectivity, China asset loading, dynamic pedestrians,
live pause fault injection and real-hardware deployment are not validated.
Camera snapshots and fixed top-view full missions are distinguished from full
route validation of every optional view. NVIDIA USD/installers/caches remain
outside this repository, subject to NVIDIA terms.
