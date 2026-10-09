# Fault supplement

The [index](faults/index.json) retains all bootstrap, protocol, startup and physical test attempts. These are actual SIM adapter negative tests, **separate from the fifteen complete Native missions**. Expected action outcome is FAILED; PASS below means the negative-test gates passed.

| Fault | Accepted attempt | Actual evidence |
|---|---|---|
| Deadline timeout | f04/timeout | Robot moved before timeout; terminal cancellation and independent stopped-state samples |
| Transport disconnect | f04/disconnect | Real observation transport loss; DDS cancellation acknowledgement and independent stop |
| Paused simulator | f05/pause | Owned simulator SIGSTOP after actual motion, unchanged simulation time, cancellation, then SIGCONT and a continuous stable stopped-state window with advancing PhysX |
| Navigator ABORT with active controller | f07/navigator-abort | Actual Navigator status 6 and actual FollowPath active status coexist; DDS acknowledges controller cancellation; fresh independent PhysX verifies stop |

The pause result deliberately keeps initial `physical_stop_verified=false`: stale, paused physics cannot prove stopping. Only the separately recorded post-resume window verifies the stopped state. The first f04 pause protocol demanded immediate stillness during normal deceleration and therefore failed; its result is preserved.

The ABORT injector forwards a real planned path to the real controller and delays the proxy goal acknowledgement beyond the BT timeout. It publishes no fake status or motion command. The experiment demonstrates the failure boundary under this explicit isolated-SIM condition, not that default Nav2 always leaks controller goals. The first f04 witness failed while serializing ROS numpy UUID elements; f06 then failed Nav2 lifecycle startup before fault injection. Both remain in the index. f07 records the actual status overlap and exact cancellation UUID.

Integration fixtures are in `challenges/01-isaac-warehouse-patrol/tests/integration/`. They require a dedicated, already authorized isolated simulation. They are not automatic CI physical tests and must never be run against REAL hardware.

The public `run_fault_supplement.py` accepts your own prior Native SIM `execution_config.json`, creates independent resets and retains every result. It refuses an existing owned SIM session and existing output directory. From the challenge directory, using the pinned ROSClaw Python:

```bash
"$ROSCLAW_SOURCE/.venv/bin/python" tests/integration/run_fault_supplement.py \
  --config YOUR_NATIVE_RUN/execution_config.json --output "$HOME/sim/fault01" \
  --fault timeout --fault disconnect --fault pause --fault navigator-abort \
  --isolated-simulation
```

The published wrapper extracts the accepted local protocol and removes machine-specific input paths. Wrapper CLI/lint checks are separate from physical protocol evidence; a final standalone public-wrapper integration run will be recorded separately, rather than retroactively attributing earlier tests to this file.

Total test wall time is not a measured stop latency. Dispatch, response acknowledgement, terminal CANCELED and physical stopping are distinct evidence. The lab DDS fallback cancels both servers only in its isolated SIM environment; upstream diagnostics do not infer goal ownership or automatically cancel all goals.
