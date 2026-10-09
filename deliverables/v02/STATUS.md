# v0.2 delivery status

Work in progress: final runtime regression is running. This document is not a release acceptance certificate.

- Runtime source frozen at `09739cb34c8a1d37ad95cfa4f146ad1e47798ec3`; upstream runtime remains `21838614bb14c39599b8acef731b2b64dad3b92a`. Documentation/test commits do not imply a different runtime was physically tested.
- Full Native plan: exactly fifteen fresh resets, five per standard/reordered/unmapped-box condition, all outcomes retained. Final metrics pending.
- [Actual fault supplement](FAULT_TESTS.md): timeout, disconnect, pause/post-resume and real Navigator-ABORT/controller-overlap gates passed, with failed attempts preserved.
- [Harness audit](HARNESS_AUDIT.md): upstream PR [#642](https://github.com/ros-claw/rosclaw/pull/642) merged at `74f83d9e40d2bbbfdd79d13b6c26835c3abfdd37`; [#644](https://github.com/ros-claw/rosclaw/pull/644), [#646](https://github.com/ros-claw/rosclaw/pull/646), [#648](https://github.com/ros-claw/rosclaw/pull/648) await required CI/merge. Their new upstream code is independently unit/fixture tested; it is not the pinned runtime used for the fifteen missions.
- Performance ablation and release evidence/video packaging pending.
- [Independent reproduction handoff](reproduction/README.md) / [中文](reproduction/README.zh.md): external engineer and second machine unassigned; third-party test NOT RUN.
- [Challenge 02 prototype/design](../../challenges/02-semantic-inspection/README.md): one development target, read-only static proposals; new-target navigation, sensor inspection, held-out experiments and fair comparisons pending.
- Hardware, complete optional security-module coverage, seekdb performance and superiority over Codex are not established.
