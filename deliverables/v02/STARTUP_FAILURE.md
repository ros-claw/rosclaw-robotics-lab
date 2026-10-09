# Frozen-source startup failure and predeclared appendix

The original fifteen-attempt plan remains immutable. `04-standard` reached the existing 1,200 s Isaac readiness timeout without PhysX state, Nav2 or a Native task. Cleanup succeeded. It is **INCOMPLETE**, never a physical PASS. Kit repeatedly reported `await_viewport: waiting for viewport handle`; the underlying trigger is unproven. No renderer setting, cache, process state or runtime source was changed to rescue the attempt.

The installed startup extension waits for a valid viewport frame handle. [Official viewport settings](https://docs.omniverse.nvidia.com/kit/docs/omni.kit.viewport.window/latest/Settings.html) describe `skipWhileInvisible=true` as suppressing updates to invisible viewports. Explicitly enabling hidden/minimized updates is an untested future workaround; this release does not claim the startup issue is fixed.

Before any appendix run, `plan-amendment.json` declared exactly one additional standard reset (`06-standard`), after the original fifteen. All sixteen outcomes and the original fifteen-run summary must be published. No further success-driven retries are authorized by that plan. The appendix declared an extension goal of five accepted missions per condition. A later provider-aborted box attempt may leave that goal unmet; the goal is reported explicitly rather than met through more retries. The user-requested independent trial coverage, accepted mission counts and retained startup/Native failures remain separate. This is an engineering regression with known limitations, not a perfect fifteen-run batch or production reliability claim.

Independent external reproduction remains NOT RUN; the user has asked for a handoff package while an engineer and second machine are unassigned.
