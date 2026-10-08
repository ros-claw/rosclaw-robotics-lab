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
