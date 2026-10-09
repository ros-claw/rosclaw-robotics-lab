# Independent reproduction handoff

**External execution: NOT RUN.** No independent engineer or second machine has been assigned. This handoff, local regressions and offline receipt replay do not constitute third-party reproduction.

Start with a separate NVIDIA GPU ARM64 host supporting Isaac Sim 6.1. Record hardware, driver, OS, existing dependencies and caches. Other architectures are separate portability tests.

1. Obtain Isaac Sim from NVIDIA, read and accept its license yourself. Use the official online assets described in the [English tutorial](../../../challenges/01-isaac-warehouse-patrol/docs/tutorial_en.md). Do not copy installers, USD assets or caches from this repository.
2. Check out the release and record the exact lab/runtime SHA, upstream SHA, Docker image ID and installed package versions. The pinned base-image manifest digest does not pin later apt downloads. A local image ID is not a registry manifest digest.
3. Configure your own supported model authentication. Never copy the developer's `.rosclaw`, `.codex`, Native `home/` or operator credentials.
4. Run `python3 scripts/lab.py doctor` in `challenges/01-isaac-warehouse-patrol`, then `python3 scripts/lab.py start headless patrol`. Use a dedicated ROS domain and free ports. Do not stop unrelated containers.
5. Follow the tutorial to start observers and submit one standard task through `lab.py task`. Use a short results path outside Git. Record installation, cached environment startup, task input, first motion, completion, retries and interventions separately.
6. Run the independent evaluator. All ordered sites, dwell, sensor, contact and final task/Memory gates must pass. Nav2 SUCCESS and the Agent's text alone are insufficient.
7. Export only whitelisted evidence, check hashes and fill in [REPORT_TEMPLATE.md](REPORT_TEMPLATE.md). Retain failed attempts. Only actual independent execution can change the report status to PASS.

`lab.py results DIRECTORY` reads the regression ledger; `lab.py stop` stops the owned environment. The optional `lab.py regression SHORT_DIRECTORY` performs fifteen fresh resets and retains every attempt. Offline replay establishes evidence consistency only, not a new simulation run. No authentication, private runtime homes, NVIDIA assets or voice models belong in the handoff.

From a release checkout, `python3 deliverables/v02/verify_evidence.py warehouse-v02-evidence.zip` checks the exact fifteen-attempt ledger, frozen configuration and runtime files, Body declarations, distinct reset identities, receipts and physical evidence replay. It is an offline consistency check, not signature attestation or third-party execution. Five deterministic metadata-binding tests cover changed configuration/checkout, wrong Body declarations and unlisted files; they do not simulate a robot.

Known startup limitation: one frozen-source attempt reached the 1,200 s Isaac viewport initialization timeout before physics/Native. It remains INCOMPLETE in the published denominator; see [startup report](../STARTUP_FAILURE.md). The original fifteen-run summary is not rewritten by the single predeclared appendix.
