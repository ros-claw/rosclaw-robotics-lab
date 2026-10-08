# Native Isaac patrol integration

The execution adapter is a daemon-owned, SIM-only **single site** Nav2 executor.
It accepts `site_id` only, resolves that ID from the measured registry, and
binds every request to the compiled Nova Carter Body hash. The Native Agent
receives the user's task and chooses individual visits. No tool executes an
entire tour. The immutable expected order is an independent acceptance contract,
not a navigation program and is not returned by the observer.

`rosclaw/prepare.py` compiles the existing eURDF/Body schemas from the composed
USD joint inventory, actual frames, Hesai LiDAR, wheel joints, map hash and Nav2
limits. It copies the operator's existing model configuration into a private
isolated home. Authentication files stay outside Git and public reports.

`rosclaw/native.py` uses ROSClaw's real chat entry point, Native Agent, Agentd,
TaskKernel, rosclawd ActionChannel and operatord. Its PTY utility and acceptance
orchestration are adapted from the pinned ROSClaw upstream MIT integration.
It sends one natural-language task. The test operator approves cards only after
validating the isolated SIM-only Body-bound configuration. It does not plan or
send navigation goals. Physical MCP declarations cannot execute motion.

The daemon uses the existing ROSClaw ROS action client and transport. It writes
Nav2 terminal status, feedback, independent PhysX trajectory/contact evidence
and fresh scan timestamps into per-action artifacts. The canonical receipt binds
the artifact SHA-256. Arrival requires position error <=0.4 m, yaw error <=0.35
rad, >=2 simulation seconds of stable dwell, speed <=0.02 m/s, angular speed
<=0.1 rad/s, zero non-floor contact pairs and complete contact observations.
An observer UUID prevents a reset inside a live mission from continuing as the
same physical run. Raw files are retained locally under ignored runtime/runs.

`patrol.verify_and_remember` replays all visit evidence, checks canonical receipt
identity, mode, state, evidence level, artifact path/hash, physical verification,
chronological order and the exact operator acceptance contract. Only then does
it publish verification through the existing RosPracticeAdapter and require
successful storage in the existing Memory. The PracticeRecorder ends successfully
only at that point. The Native harness separately checks TaskKernel SUCCEEDED.

## Explicit differences from the official baseline

The original official and calibrated three-goal configurations remain available.
`config/patrol_navigation_params.yaml` is a separate configuration:

* Keeps measured AMCL alpha values 0.001 and goal tolerance 0.10 m.
* Sets AMCL update_min_d/update_min_a to zero, so actual stationary scan updates
  continue instead of presenting an old localization observation as fresh.
* Removes the two local costmap 2D laser layers whose required scan publishers
  were absent in the loaded Isaac 6.1 scene. Hesai PointCloud2 and derived /scan
  remain the actual obstacle/localization sensors; no replacement 2D scans are
  fabricated.
* Binds controller odometry to actual /chassis/odom.
* Publishes full local/global costmaps at 2 Hz of simulation time to provide
  bounded observable freshness. This changes publication, not obstacle data.
* Sets bridge nodes to use_sim_time=true.

The read-only native probe keeps the upstream graph, parameter, lifecycle, QoS,
TF and subscription model, while recognizing named NVIDIA costmap plugin
parameters. It subscribes to actually offered streams; subscriber-only endpoints
remain in the full graph. Transient-local occupancy grids retain their latched
policy. Time-domain checks include data publishers and lifecycle nodes; auxiliary
launch/parameter-only nodes are preserved separately. These rules avoid treating
an inactive subscription or optional static debug grid as a live sensor. Missing
required sensor/clock/TF evidence still cannot pass the independent verifier.

The first real Native pilot invoked fail-safe and submitted no navigation action;
`reports/native-pilot-preflight.json` records that result. Subsequent success must
be established by actual canonical receipts and physical evidence, not this
implementation description.

## Recovery accounting

The bounded adapter allows at most six Nav2 `number_of_recoveries` operations
per goal, within the existing 420-second wall timeout. Nav2 counts individual
local/global costmap clearing operations as recoveries; six operations do not
mean six complete behavior-tree retry cycles. The packaged official tree has
six outer retries and context-specific inner recoveries. The earlier two-operation
budget cancelled a real Home attempt after three clearing operations; that failed
run remains in `native-recovery-limit-failure.json`. A separate single-goal Home
probe subsequently succeeded with no recovery. This motivates the declared budget,
not a relaxation of physical acceptance. The compiled Body angular envelope is
1.2 rad/s to include the actual official recovery behavior limit; the DWB controller
limit remains 0.7 rad/s.

## Unmapped obstacle and layer order

The first unmapped-box probe observed thousands of actual pointcloud hits and
physical avoidance, but every sampled local master-costmap cell in the box region
remained zero. It therefore failed the stronger obstacle-evidence contract.
The inherited local plugin order was voxel → inflation → static. In Nav2's
[StaticLayer implementation](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_costmap_2d/plugins/static_layer.cpp),
a rolling static layer with `use_maximum=false` writes static-map values over the
master grid. This supports the diagnosis that the last static layer erased dynamic
obstacle costs. The patrol configuration now orders static → voxel → inflation;
the original NVIDIA baseline configuration remains intact. Subsequent verification
must show actual local occupied cells, not infer success from this diagnosis.
