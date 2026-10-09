# Retained failures and what changed

| Evidence | Observed behavior | Result / correction |
|---|---|---|
| `native-pilot-preflight.json` | Missing 2D streams, stale stationary AMCL, clock classification | No motion. Dedicated patrol configuration and actual-stream probe; baseline unchanged. |
| `native-integration-trial.json` | Four physical arrivals and Memory succeeded, TaskKernel closed after first visit | Whole mission FAIL. Per-visit evidence no longer appears as a whole-task artifact; final timestamp gate added. |
| `native-follow-preflight-failure.json` | Required streams 1.8–2.2 wall seconds old | No motion. Probe now enforces declared 3 wall/1 simulated second freshness consistently with independent LiDAR. |
| `native-context-preflight-failure.json` | First proposal rejected CONTEXT_HASH_MISMATCH | No motion. Removed added CLI workspace flag; future run homes outside Git. Exact root cause remains unproven. |
| `native-recovery-limit-failure.json` | Three visits passed; Home cancelled at recovery count 3 | Whole mission FAIL. Two-operation budget interrupted clearing recovery; declared six-operation bound, unchanged physical requirements. Separate Home probe succeeded; subsequent full reset passed with zero recovery. |
| `native-camera-switch-failure.json` | Runtime projection switch stopped Isaac clock updates | No motion. Runtime view switching removed; choose a fixed display camera at launch. |

All failed raw runs remain locally available. None created a successful patrol
Memory record merely from Agent prose. Successful reports are separate records;
these failures remain part of the delivery and must not be excluded when discussing
reliability. Three passing resets do not establish a statistical success rate.

The first unmapped-box case passed arrival checks and stored a generic patrol
experience, but failed the stronger independent local-costmap obstacle contract.
`unmapped-box-arrivals-only.json` and `unmapped-box-local-costmap-failure.json`
show both scopes. The local static layer was placed after voxel/inflation layers;
ordering static → voxel → inflation restored actual occupied cells. Future explicit
obstacle cases additionally require this evidence before success Memory is allowed.

The first DDS cancellation test physically stopped the robot, but serializing
uint8 UUID bytes failed, so its acknowledgement was not reviewable; it is retained
as FAIL. Converting each byte to int and requiring a successful DDS acknowledgement
fixed the tested path. An archive hash check also caught a live-append race between
hashing and reading a raw trajectory file. The packager now snapshots each file
once, hashes those bytes and archives the same bytes; all four final archives pass.

Multiview recording retains an interrupted no-motion preflight and a failed Native
run separately (`multiview-preflight-interrupted.json`,
`multiview-native-interrupted.json`). The latter physically passed Entry, then
Nav2 aborted Shelf on a FollowPath acknowledgement timeout. Physical stopping was
not verified; this is a cancellation defect, not a successful fail-safe test.
The new profile gives BT acknowledgement and cancellation requests a bounded
1000 ms deadline (previously 20 ms), and the DDS abort fallback cancels both
NavigateToPose and FollowPath. Arrival, freshness, dwell and collision thresholds
are unchanged. Parameter meaning: [Nav2 Jazzy official documentation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_bt_navigator/).

`multiview-startup-failures.json` additionally records a rejected Nav2 lifecycle
startup, a rosbridge readiness race, and an expired access-only model login. No
robot task completed in these attempts. The recording now waits for rosbridge;
fixture preparation can replace an expired Codex access token from the current
local Codex login only when the account matches, with a minimum 30-minute validity
window. Refresh tokens are not copied and global credentials are unchanged.
Authentication reference: [official OpenAI documentation](https://developers.openai.com/codex/auth/).

The first dual-server DDS disconnect regression also failed: using a one-second
per-server discovery wait prevented discovery, neither cancellation was
acknowledged, and physical stopping was false. The failed public summary is
`fault-dual-cancel-multiview.json`. Both clients are now created before discovery
and share the original five-second total deadline. This correction does not
change positive navigation or any physical acceptance threshold.

The corrected dual-server disconnect regression passes in a separate reset
(`fault-dual-cancel-multiview-retry.json`): real motion was observed, DDS returned
actual cancelling UUIDs for both NavigateToPose and FollowPath, the action failed
as required, and independent PhysX verified stopping. Total test duration was
14.51 wall seconds, not a measured stop latency. The precise navigator-ABORT
orphan-controller scenario has not been separately injected after this fix.
The positive multiview Native mission passes all four visits, final Memory,
TaskKernel closure, and the strengthened unmapped-box contract.
