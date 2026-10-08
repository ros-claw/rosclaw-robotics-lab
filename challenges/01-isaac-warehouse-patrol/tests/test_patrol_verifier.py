"""Synthetic unit inputs validate the gate, never establish simulation success."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "evaluator"))
from patrol import verify_visit


class PatrolVerifierTests(unittest.TestCase):
    def fixture(self):
        samples = [
            {
                "physics_body_path": "/chassis",
                "wall_time": 100 + i * 0.2,
                "sim_time": i * 0.2,
                "timeline_playing": True,
                "collision_observer_complete": True,
                "collision_count": 0,
                "contact_errors": [],
                "physics_transforms_xyzw": [[0, 0, 0, 0, 0, 0, 1]],
                "linear_velocity_xyz": [0, 0, 0],
                "angular_velocity_xyz": [0, 0, 0],
            }
            for i in range(15)
        ]
        return (
            samples,
            {"x": 0, "y": 0, "yaw": 0},
            {"status": 4, "error_code": 0, "wall_time": 100},
            {
                "valid": True,
                "wall_time": samples[-1]["wall_time"],
                "stamp": samples[-1]["sim_time"],
            },
        )

    def test_complete_visit_passes(self):
        self.assertEqual(
            verify_visit(*self.fixture(), body_path="/chassis")["verification_status"],
            "PASS",
        )

    def test_cancellation_error_zero_cannot_pass(self):
        values = list(self.fixture())
        values[2]["status"] = 5
        self.assertEqual(
            verify_visit(*values, body_path="/chassis")["verification_status"], "FAIL"
        )

    def test_collision_fail_closed(self):
        values = list(self.fixture())
        values[0][4]["collision_count"] = 1
        self.assertEqual(
            verify_visit(*values, body_path="/chassis")["verification_status"], "FAIL"
        )

    def test_missing_collision_sensor_is_unknown(self):
        values = list(self.fixture())
        values[0][4]["collision_observer_complete"] = False
        self.assertEqual(
            verify_visit(*values, body_path="/chassis")["verification_status"],
            "UNKNOWN",
        )

    def test_stale_lidar_is_unknown(self):
        values = list(self.fixture())
        values[3]["wall_time"] -= 10
        self.assertEqual(
            verify_visit(*values, body_path="/chassis")["verification_status"],
            "UNKNOWN",
        )

    def test_nonfinite_physics_is_unknown(self):
        values = list(self.fixture())
        values[0][4]["sim_time"] = float("nan")
        self.assertEqual(verify_visit(*values, body_path="/chassis")["verification_status"], "UNKNOWN")

    def test_observer_reset_cannot_pass(self):
        values = list(self.fixture())
        for sample in values[0]:
            sample["observer_id"] = "before"
        values[0][-1]["observer_id"] = "after"
        self.assertEqual(verify_visit(*values, body_path="/chassis")["verification_status"], "FAIL")

    def test_fast_flyby_cannot_pass(self):
        values = list(self.fixture())
        for sample in values[0]:
            sample["linear_velocity_xyz"][0] = 0.3
        self.assertEqual(
            verify_visit(*values, body_path="/chassis")["verification_status"], "FAIL"
        )


if __name__ == "__main__":
    unittest.main()
