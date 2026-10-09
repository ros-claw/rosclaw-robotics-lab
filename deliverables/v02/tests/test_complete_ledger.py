"""Prevent dropping startup failures or retroactively declaring extra runs."""

import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "ledger_verify", Path(__file__).parents[1] / "verify_evidence.py"
)
verify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(verify)


def records():
    rows = [
        dict(
            attempt=f"{i:02d}-{c}",
            condition=c,
            repetition=i,
            status="PASS",
            started_wall_time=i * 100,
        )
        for i in range(1, 6)
        for c in ["standard", "reordered", "unmapped-box"]
    ]
    next(r for r in rows if r["attempt"] == "04-standard")["status"] = "INCOMPLETE"
    rows.append(
        dict(
            attempt="06-standard",
            condition="standard",
            repetition=6,
            status="PASS",
            started_wall_time=600,
        )
    )
    return (
        rows,
        {"declared_wall_time": 450},
        {"attempts": 15, "passes": 14, "failures_and_incomplete": ["04-standard"]},
    )


class LedgerTests(unittest.TestCase):
    def test_predeclared_appendix_keeps_startup_failure(self):
        verify.check_plan(*records())

    def test_additional_native_failure_is_retained_without_forcing_success(self):
        rows, amendment, original = records()
        next(r for r in rows if r["attempt"] == "04-unmapped-box")["status"] = "FAIL"
        original["passes"] = 13
        original["failures_and_incomplete"].append("04-unmapped-box")
        verify.check_plan(rows, amendment, original)
        self.assertEqual(sum(r["status"] == "PASS" for r in rows), 14)

    def test_failure_cannot_be_dropped_from_denominator(self):
        rows, amendment, original = records()
        rows = [r for r in rows if r["status"] == "PASS"]
        with self.assertRaisesRegex(ValueError, "original fifteen"):
            verify.check_plan(rows, amendment, original)

    def test_retroactive_appendix_is_rejected(self):
        rows, amendment, original = records()
        amendment["declared_wall_time"] = 601
        with self.assertRaisesRegex(ValueError, "before execution"):
            verify.check_plan(rows, amendment, original)

    def test_original_failure_summary_cannot_hide_failure(self):
        rows, amendment, original = records()
        original["failures_and_incomplete"] = []
        with self.assertRaisesRegex(ValueError, "failed attempts"):
            verify.check_plan(rows, amendment, original)


if __name__ == "__main__":
    unittest.main()
