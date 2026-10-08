"""Synthetic unit fixtures cannot be used as simulation acceptance evidence."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "evaluator"))
from baseline import evaluate


class BaselineTests(unittest.TestCase):
    def fixture(self):
        rows = [
            {
                "wall_time": i + 1.0,
                "timeline_playing": True,
                "physics_transforms_xyzw": [[x, y, 0, 0, 0, 0, 1]],
            }
            for i, (x, y) in enumerate([(-4, -1), (-4, 1), (-6, 1)])
        ]
        goals = "\n".join(f"[{i + 1}.0] Result: error_code=0" for i in range(3))
        nav = "\n".join(f"[{i + 1}.0] [bt_navigator]: Goal succeeded" for i in range(3))
        return rows, goals, nav

    def test_missing_independent_state_is_unknown(self):
        _, goals, nav = self.fixture()
        self.assertEqual(evaluate([], goals, nav)["accuracy_status"], "UNKNOWN")

    def test_nav_success_does_not_override_physical_error(self):
        rows, goals, nav = self.fixture()
        rows[-1]["physics_transforms_xyzw"][0][0] += 0.6
        self.assertEqual(evaluate(rows, goals, nav)["accuracy_status"], "FAIL")

    def test_stale_physics_is_unknown(self):
        rows, goals, nav = self.fixture()
        for row in rows:
            row["wall_time"] += 10
        self.assertEqual(evaluate(rows, goals, nav)["accuracy_status"], "UNKNOWN")

    def test_error_zero_without_nav_success_is_fail(self):
        rows, goals, _ = self.fixture()
        self.assertEqual(evaluate(rows, goals, "")["accuracy_status"], "FAIL")


if __name__ == "__main__":
    unittest.main()
