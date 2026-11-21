from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from evaluator.controller_run import first_settled_index


def test_physx_stop_can_follow_odom_stop_and_requires_continuous_samples():
    def sample(i, angular):
        return {"sim_time_s": i / 20, "host_mono_ns": str(i * 50_000_000),
                "linear_angular_velocity": [0.02, 0, 0, 0, 0, angular]}

    rows = [sample(i, 0.09 if i < 12 else 0.07) for i in range(40)]
    assert first_settled_index(rows, 700_000_000) == (12, 27)
    assert first_settled_index(rows[::5], 700_000_000) is None
