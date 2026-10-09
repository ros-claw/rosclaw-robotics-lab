# Warehouse Patrol v0.2 delivery status

Frozen-source regression completed: **14/16 attempts accepted**. All original fifteen attempts and the single predeclared standard appendix are retained. The original fifteen-run summary remains FAIL; `04-standard` is startup INCOMPLETE, and `04-unmapped-box` is a provider-aborted partial task/outer-timeout FAIL. No failed task was manually resent or replaced by a later success.

| Condition | All attempts | Native started | Complete PASS |
|---|---:|---:|---:|
| standard | 6 | 5 | 5 |
| reordered | 5 | 5 | 5 |
| unmapped-box | 5 | 5 | 4 |

The extension goal of five accepted missions per condition is **False**. This is a completed engineering regression with known limitations, not an all-tests-passed claim. [Startup](STARTUP_FAILURE.md), [provider failure](PROVIDER_FAILURE.md), [complete metrics](regression-metrics.json) and [full report](IMPLEMENTATION_REPORT.zh.md) explain the boundaries.

- Runtime lab `09739cb34c8a1d37ad95cfa4f146ad1e47798ec3`, upstream `21838614bb14c39599b8acef731b2b64dad3b92a`. Documentation/test commits have identical runtime file hashes; they do not imply the new upstream was physically substituted.
- Five upstream PRs [#642](https://github.com/ros-claw/rosclaw/pull/642), [#644](https://github.com/ros-claw/rosclaw/pull/644), [#646](https://github.com/ros-claw/rosclaw/pull/646), [#648](https://github.com/ros-claw/rosclaw/pull/648), [#650](https://github.com/ros-claw/rosclaw/pull/650) merged after required CI. See [exact merge records](upstream-merged-prs.json) and [audit](HARNESS_AUDIT.md).
- [Fault supplement](FAULT_TESTS.md) retains earlier failed protocols/startups and actual timeout/disconnect/pause/ABORT evidence. The public CLI rerun outcomes are `{'navigator-abort': 'PASS', 'disconnect': 'PASS', 'timeout': 'PASS', 'pause': 'PASS'}`; [full index](faults/public-wrapper-index.json). Negative-test PASS means the stopping gates passed, not that its deliberately failed action succeeded.
- [Performance](PERFORMANCE_PROTOCOL.md): six model-free camera-profile trials; [outcomes](performance-results.json), [CPU/GPU observations](resource-summary.json). No pure kernel or isolated remote inference timing claim.
- 60/180/510 s synchronized videos are historical fifth-mission footage, separate from this frozen regression; jump/time/speed labels and acquisition/output FPS distinction are retained.
- [Independent handoff](reproduction/README.md) / [中文](reproduction/README.zh.md): no engineer/second machine assigned; **external reproduction NOT RUN**. Offline archive consistency is not new physical acceptance or a signature.
- [Challenge 02](../../challenges/02-semantic-inspection/README.md): actual read-only development proposals, grouped holdout manifest and fair protocols prepared. New-target motion, sensor inspection, held-out and Agent/development comparisons remain pending. No superiority over Codex is established.
- Runtime uses SQLite and optional firewall integration is disabled; hardware, all optional security modules and seekdb performance are not validated.

Release source, evidence, checksum/reproduction bundle and three videos are distributed through [warehouse-patrol-v0.2.0](https://github.com/ros-claw/rosclaw-robotics-lab/releases/tag/warehouse-patrol-v0.2.0). Use the release asset checksums; the preparation ZIP remains an earlier handoff snapshot.
