# Performance measurement protocol

Report four sources of work separately. Their wall intervals overlap and must not be added as independent costs.

| Component | Observation | Interpretation limit |
|---|---|---|
| Physics / Kit | Advancing independent PhysX simulation timestamps versus wall time; owned Kit CPU user/system ticks | Kit CPU also includes rendering, sensor and Python work; this is not a pure physics timer |
| Rendering / capture | Two independent resets per camera profile, reversed order in repetition two; actual timestamped capture groups | Composite rendering/capture effect; no isolated GPU render kernel timing or motion interpolation |
| ROS / Nav2 | Project-labelled container cgroup CPU deltas, action start/finish and cancellation evidence | Containers overlap Kit and Agent work; CPU is not network latency or pure navigation compute |
| Agent / tools | Input-to-first-motion, actual model turns/tool calls, SDK usage, ordered inter-goal gaps and bound process CPU | Gaps include model, tool, approval, IPC and orchestration waits; remote inference is not local process CPU |

The fifteen Native missions use one 1280×720 follow viewport and no PNG capture. A separate model-free single-site SIM adapter experiment uses 1920×1080 for every profile: viewport without PNG, viewport with PNG, and synchronized three-view PNG. Two repetitions reverse profile order to reduce cache/drift effects. Every trial resets the scene and retains failure. Route distance and physical arrival/contact gates are checked; profiles are not assumed to produce identical trajectories.

The no-PNG base still renders the viewport and RTX LiDAR. It is **not** a headless pure-physics baseline. Three-view capture changes rendering and encoding workload together. Two repetitions are exploratory and do not establish a causal per-frame decomposition or a production benchmark. The 1280×720 Native trials cannot be directly subtracted from the 1920×1080 ablation as an Agent cost.

Resource samples are collected every ten seconds, including failed startup intervals. CPU ticks are converted with recorded clock ticks per second; container CPU is in microseconds. Report average cores as CPU seconds / wall seconds. GPU utilization/power is host-wide sampled telemetry, not exclusive attribution to Kit or an inference model. Missing or unsupported metrics remain missing. Compare counters only for the same PID/container/observer identity and increasing timestamps.

Installation, cached environment readiness, input-to-first-motion, physical action intervals and whole-attempt wall time are distinct. SDK cost fields are estimates from the SDK metadata, not verified charges. Neither video encoding FPS nor repeated output frames are actual camera acquisition FPS. Historical three-view video performance belongs to its historical fifth mission, separate from this frozen regression.

Source-controlled final results and analysis will accompany the release; raw whitelisted observations belong in the evidence archive. No ROSClaw-versus-Codex speed advantage follows from this experiment.
