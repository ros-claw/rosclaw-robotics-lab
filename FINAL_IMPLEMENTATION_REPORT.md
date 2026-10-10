# Warehouse loading-area inspection — implementation and post-merge acceptance

2026-10-10. FINAL — actual post-merge upstream build and required live/offline acceptance passed.

The existing warehouse challenge now executes one real Chinese business task through Native Agent → Agentd/Operator → rosclawd → Nav2 → Isaac Sim → independent verification → canonical receipts → Practice/Memory → TaskKernel. It selects the forklift near shelves using known facility geometry, generates safe viewpoints, inspects actual raw PointCloud2, makes bounded additional observations, returns to the measured start and reports findings and uncertainty. Motion and task completion use the existing runtime.

## Evidence-based outcome

All 12 independent resets passed (A 3/3 · B 3/3 · C 3/3 · D 3/3); 27 verified visits, maximum position error 0.172721 m, zero non-floor contacts. Across all loading attempts on this same final lab source, including calibration/interrupted batches and excluding legacy compatibility, outcomes remain 20 PASS / 1 FAIL; the final protocol result does not erase those failures.

| Attempt | Case | Task | Result | Coverage | Views / actual new cells | Native wall (s) | Measured SIM span (s) | Model turns | Tool calls |
|---|---|---|---|---:|---|---:|---:|---:|---:|
| l04ia01 | A | PASS | OBSTRUCTED | 100.0% | 238 | 162.1 | 122.0 | 7 | 6 |
| l04ia02 | A | PASS | OBSTRUCTED | 100.0% | 238 | 173.7 | 116.8 | 7 | 6 |
| l04ia03 | A | PASS | OBSTRUCTED | 100.0% | 238 | 158.3 | 120.1 | 7 | 6 |
| l04ib01 | B | PASS | OBSTRUCTED | 81.5% | 190 / 4 | 210.6 | 137.0 | 9 | 8 |
| l04ib02 | B | PASS | OBSTRUCTED | 81.5% | 191 / 3 | 226.5 | 113.4 | 9 | 8 |
| l04ib03 | B | PASS | OBSTRUCTED | 81.5% | 192 / 2 | 235.3 | 114.7 | 9 | 8 |
| l04ic01 | C | PASS | OBSTRUCTED | 100.0% | 200 / 38 | 435.2 | 80.7 | 10 | 9 |
| l04ic02 | C | PASS | OBSTRUCTED | 100.0% | 202 / 36 | 267.6 | 108.4 | 9 | 8 |
| l04ic03 | C | PASS | OBSTRUCTED | 100.0% | 200 / 38 | 255.8 | 109.1 | 9 | 8 |
| l04id01 | D | PASS | UNKNOWN | 0.0% | none (refusal) | 85.4 | 46.3 | 6 | 5 |
| l04id02 | D | PASS | UNKNOWN | 0.0% | none (refusal) | 91.3 | 49.4 | 7 | 6 |
| l04id03 | D | PASS | UNKNOWN | 0.0% | none (refusal) | 96.2 | 56.8 | 6 | 5 |

SIM spans use actual physics samples bracketing each Native window, including model/operator waiting; individual boundary overhangs are recorded in performance-windows.json. No time interpolation.

Active-observation acceptance uses case C: second views added 38, 36 and 38 measured cells. B3 added only two cells on its second view, below the three-cell capability threshold; it is not counted as an active-observation success. Its task PASS rests on actual obstacle evidence, a correct OBSTRUCTED conclusion, verified return and Memory.

Final acceptance rows belong to a fourth pre-frozen engineering batch: lab `d1a1db271ae0030ea47df415d890751dcfd205fb`, actual merged ROSClaw and rebuilt Native `9862058c445c88156b9080832b5dd803c75279ae`. Cases A/B use different start poses; C/D start nearer the other forklift. The task is identical across cases, with no fixture label, seed, coordinate or expected answer in the user input. All failures remain in the ledger, without replacement or retrospective threshold changes.

No live CLEAR acceptance is claimed for this batch; CLEAR behavior is covered by unit tests. The normal warehouse is not presumed CLEAR. Its inspected entrance lies close to the existing forklift: actual outside-ROI sensor returns affect the full footprint and establish blockage under the declared conservative clearance. B separately records extra occupancy. C must start with genuinely partial coverage and gain ≥3 measured cells in a second view. D must make zero inspection visits and report UNKNOWN under a restricted immutable observation zone. Correct OBSTRUCTED or justified UNKNOWN can complete the task successfully.

## Implementation and boundaries

Known facilities are captured from the actual composed Stage before reset fixtures are applied. Structural entity grouping includes shelves, forklifts, cart and pallets; geometric box distance resolves “near shelves.” The suffix Forklift_01 is not the selector. Swapped-name and ambiguous-relation tests preserve that boundary. The independently expected target in this official layout happens to be Forklift_01.

The declared local work patch is generated from shelf-front and forklift bounds. This is a task-specific inspection contract in one known warehouse, not an arbitrary language planner or proof of unseen-layout generalization. Predicted visibility ranks proposals only. Fresh state, immutable proposal/source/Body bindings, Nav2 path computation and complete swept-footprint checks gate each actual navigation request.

Raw PointCloud2 retains original bytes, field layouts, SHA256 and measurement-time TF. Independent replay decodes and transforms those bytes again. Ground is z=0±0.08m; the measured obstacle band is 0.10–0.80m. A 0.2m cell receives free-space credit only from finite measured rays in all three declared height bands. No-return and unknown space grant no free credit. A measured halo around the region prevents cropping away nearby obstacles needed for full-footprint clearance.

The actual asymmetric navigation footprint has a 0.65647m enclosing radius. Inspection uses 0.75m plus a half-cell diagonal guard, rounded conservatively to a five-cell square stencil. CLEAR requires at least 70% measured ROI coverage and a verified free path. OBSTRUCTED requires a measured occupancy clearance barrier; unknown cells cannot create one. UNKNOWN preserves insufficient or invalid evidence. At most two inspection viewpoints are allowed, enforced in the immutable calibrated profile; the extra-object flag and passage result remain separate.

These finite-resolution rays do not prove every thin object absent. Objects below 0.10m are not cleared. RGB cameras are display-only; there is no VLM identification, goods/defect recognition or industrial safety certification. D covers authorization-zone refusal, not every narrow-space condition. Scene truth, sensor evidence and evaluation remain distinct. Sanitized SDK audits contain task inputs, tool names/times and file-access verdicts/hashes. Loading tasks read only the official skill; the two legacy compatibility tasks additionally read/grep their exact successful verification reports after creation. The first audit rule incorrectly rejected those legitimate reads; its FAIL record is retained separately, without relabeling task results. The same-UID development setup is not a malicious-file-access sandbox.

## P0 actual scene and sensor audit

The visible fork Meshes have collision disabled, but both forklifts already have valid invisible guide `S_ForkliftFork/collision_box` proxies. The final playing Stage contains 1679 bounded schema rows and 26 raw fork-related prims. Both forks pass 32/32 actual visible-surface overlap queries; independent control contacts name each corresponding proxy. No proxy was added and no official NVIDIA asset modified.

The initial high-drop near-shelf control struck a pallet/rack and is retained as confounded, not credited as fork proof. The lower near-fork control also contacted a neighboring pallet; the complete paths and independent overlap/ray checks are retained. This is current-asset sampled collision evidence, not a universal fork-geometry equivalence certificate.

The actual cart Prim is `.../Warehouse_Empty_small_realtime/SM_PushcartA_02_22`; its child `SM_PushcartA_02` is colliding. Cart identity/cargo was not inferred from a screenshot. The actual odometry is `/chassis/odom`; `/odom` has no active publisher. Raw cloud, scan, TF, clock and local costmap were audited independently.

Three actual low boxes, 0.15/0.35/0.70m high, produced measured ≥0.10m endpoints [39,38,38], [327,323,320], [691,692,690] in their bounds across three accepted frames. Initial TF warm-up errors remain in the record. This supports only the tested distances, poses and declared height band. See the [full P0 audit](challenges/02-semantic-inspection/docs/P0_AUDIT.zh.md).

## Retained calibration failures

Seven Native/startup calibration attempts are separate from the formal denominator. l04pa01 completed its physical UNKNOWN/return/Memory sequence but independent acceptance failed because the source/build stamps differed. A pre-start guard now rejects such a mismatch. l04pa02 was accepted by the old boundary-cropped algorithm, but its CLEAR cannot support the final capability claim. Independent boundary reanalysis changes it to OBSTRUCTED; the original file is preserved rather than rewritten.

l04pb01 detected an actual temporary obstacle and reported blockage, but its second observation added only one cell; it is not credited as successful active observation. l04pc01 selected the correct forklift from another start but still saw 97.9% initially; it failed the intended occlusion case. l04pc02 never began a task because integer initial_pose fields were rejected by AMCL; parameter types were normalized. l04pc03 then passed the corrected actual occlusion test, improving 84.45% to 100% with 37 new cells, returning safely and completing Memory. l04pd01 correctly refused all unauthorized observations and completed a verified return with UNKNOWN.

P0 also retains the unsupported BBoxCache keyword startup and a delayed-supervisor cleanup race. The latter is fixed by checking simulator PID ownership before an ERR cleanup. Neither environment failure is presented as a task PASS. No human steering was used to rescue the successful formal missions; actual interventions and timeouts, if any, are listed separately in the complete ledger.

The first frozen batch (lab fe65ecd) was stopped after four actual resets: l04fa01 PASS, l04fa02 PASS, l04fa03 FAIL and l04fb01 PASS. Eight planned resets never started. The failure exposed an initial rotation whose full swept footprint intersects inflated costmap cells. Static endpoint safety alone was insufficient. The replacement protocol uses nominal starts that permit safe rotations; the failed start remains a separate safety regression.

Additional pilots l04qa03 (startup readiness timeout), l04qa04 (expired return proposal) and l04qd05 (unsafe computed return rotation, stopped before dispatch) remain FAIL. Actual Nav2 previews and every rejected snapshot are now retained before publishing observation proposals. Proposal TTL starts after preview computation, and background preview work cannot hold the dispatch catalog lock indefinitely. The qd05 failure is not relabeled as a successful refusal: it prevented unsafe motion but did not complete Memory. D acceptance covers a predeclared restricted observation zone with a safe verified return. No gate was lowered and no unsafe motion was forced.

The Native input directory also no longer retains the legacy full Stage inventory copy. Only the facility prior captured before fixture insertion is provided for this task. This reduces accidental fixture-truth exposure; the same-UID boundary still is not adversarial isolation.

Intermediate calibration l04qc06 passed on a092aad: 84.45% then +37 cells to 100%. After the catalog-ID correction, final-source l04qc07 passed on d1a1db2: 83.61% then +39 cells to 100%, truthful OBSTRUCTED, verified return, Memory/TaskKernel and full recording acceptance. The final-source B open-start pilot l04qb08 also passed before the fourth protocol was frozen; the final batch begins only after that pilot’s owned cleanup.


The second frozen batch on a092aad stopped after A1–A3 completed PASS (nine planned resets did not start). Code review found the catalog mixed retired IDs with newly issued IDs. It is retained separately, not included in the final denominator. Final d1a1db2 publishes only freshly registered IDs and checks complete candidate/evidence hashes before exposing them. The new regression rejects retired IDs, duplicate IDs, target tampering and evidence changes.

The third frozen batch, already on final source d1a1db2, retains seven started resets: three A PASS, two B PASS, B3 FAIL and one C PASS with recording PASS. Five planned resets never started. B3's temporary obstacle left insufficient initial rotational clearance; the fresh dispatch sweep rejected motion before a Nav2 request. This is a real task failure on the final implementation, not relabeled as task success. The fourth protocol changes B starting poses to a pre-calibrated open area, with source, perception thresholds and safety gates unchanged. Thus any final 12/12 result applies only to that final protocol; it does not erase the current-source failure or establish unrestricted-start reliability. All current-source attempts are visible in the unique attempt ledger.

## Upstream PR and actual post-merge build

[ROSClaw PR #660](https://github.com/ros-claw/rosclaw/pull/660) adds a generic read-only CLEAR/OBSTRUCTED/UNKNOWN measured-passage classifier with explicit evidence validity, conservative clearance and linear-time integral counts. No warehouse coordinate, forklift identifier or new runtime is upstreamed. A missing-module regression was captured before the implementation; 12 focused tests cover evidence, unknowns, barriers, clearance and invalid/bounded inputs.

All 13 required checks passed, including Python 3.11/3.12/3.13, full regression, type/lint, boundary, deployment and acceptance checks. The PR was normally squash-merged at 2026-10-10 07:49:12 UTC as `9862058c445c88156b9080832b5dd803c75279ae`, without an administrator bypass.

After merge, actual origin/main was fetched into a new isolated worktree and Native rebuilt there. Source worktrees and trial HOME directories were isolated; the installed Python venv and Node dependency cache were reused and recorded. This is not a fresh-OS reproduction. Its build stamp is exactly the merge SHA. Pre-merge PR-head results are calibration only. Post-merge results: ROS Connector 381 passed/10 deselected; observation/projection/session tests 25 passed; focused inspection plus session lifecycle 19 passed (overlap with other suites, not additive unique counts). Lab semantic tests 51 passed; CI collects pytest tests explicitly and pins the actual merged upstream SHA.

C01 standard four-site task: **PASS** (l04c01), actual upstream source/build `9862058c445c88156b9080832b5dd803c75279ae`. See its original acceptance and source-freeze in the formal evidence archive.

Legacy C02 dynamic shelf + region LiDAR: **PASS** (l04oldc02), actual upstream source/build `9862058c445c88156b9080832b5dd803c75279ae`. See its original acceptance and source-freeze in the formal evidence archive.



Traceability: PR #660 → merge 9862058c → fetched main → Native build 9862058c → unit/Harness regressions → new frozen physical batch plus C01/legacy C02. [Runtime bindings and merged PR records](deliverables/v04/) preserve checks, timestamps and hashes.

## Video, reproduction and release

The 90-second video depicts final-source calibration l04qc07, separate from the 12-trial denominator: actual synchronized third-person, robot-forward and close-top views; raw measured cloud and full-footprint deficit cells; 83.61% to 100% coverage (+39 cells), OBSTRUCTED, verified return and Memory/TaskKernel. Median camera acquisition 1.135 fps, maximum gap 2.085s, encoding 8 fps, middle wall-time compression x3.657, source SIM/wall ratio 0.404. Opening/ending holds 6/8s, no interpolation. Full decode and visual checks passed. RGB is display-only; orange cells derive only from actual measured occupancy.

Use the [English tutorial](challenges/02-semantic-inspection/docs/LOADING_TUTORIAL.md) or [中文教程](challenges/02-semantic-inspection/docs/LOADING_TUTORIAL.zh.md). The [v04 delivery directory](deliverables/v04/) includes frozen fixture/protocol hashes, every attempt ledger, P0 audit summary, post-merge tests, sanitized single-input audits and video provenance. Release evidence uses an explicit whitelist; original USD assets, private HOME, raw model reasoning, authentication and operator keys are excluded.

[v0.4 release](https://github.com/ros-claw/rosclaw-robotics-lab/releases/tag/loading-area-inspection-v0.4.0). Core video, P0, calibration/failure evidence and actual selected camera-source assets were re-downloaded and SHA256-verified. Final trial/compatibility evidence has separate frozen replay records. A final complete release-asset download verification receipt accompanies the release. External live reproduction remains NOT RUN.

Archive replay checks file hashes, frozen source, original raw point clouds, canonical receipts, physical dwell/contact/return, sensor results, independent case truth and Memory ordering. It is offline consistency, not a new live simulation or third-party reproduction.

## Historical scope and team decisions

Previous [v02](https://github.com/ros-claw/rosclaw-robotics-lab/releases/tag/warehouse-patrol-v0.2.0) and [v03](https://github.com/ros-claw/rosclaw-robotics-lab/releases/tag/semantic-observation-v0.3.0) remain intact. v02 records 16 attempts (14 complete PASS, one startup incomplete and one provider failure); v03 records five evolving-code attempts (one FAIL, four PASS), separately from its C01 compatibility run. Their metrics and videos are not mixed into this frozen batch.

An independent engineer and second machine have not been arranged. Independent live reproduction remains NOT RUN. No fair ROSClaw-versus-Codex comparison was run and no superiority claim is made. The current evidence supports controlled task execution in this known SIM warehouse, including measured feedback, bounded active observation and safe refusal. Next team decisions should prioritize independent reproduction and a new layout/target protocol before dynamic equipment or hardware testing.

The initial public replay failed on C01 because the original live evaluator requires the private mission SQLite DB and absolute local artifact paths, which are deliberately not distributed. The public C01 adapter instead invokes its original frozen physical verifier, checks unchanged canonical receipt/path/hash bindings, and validates the exported actual TaskKernel/Memory timestamps; it synthesizes no private DB or session. The failed replay log is retained alongside the corrected replay result.
