#!/usr/bin/env python3
"""Independent loading mission replay with original canonical receipt/Memory gates."""

import argparse
import json
from pathlib import Path
from evaluate import evaluate
from loading_executor import LoadingMemoryExecutor

p = argparse.ArgumentParser()
p.add_argument("--directory", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
try:
    result = evaluate(a.directory.resolve(), checker_type=LoadingMemoryExecutor)
except Exception as e:
    result = {
        "status": "FAIL",
        "error": str(e),
        "evidence_domain": "SIMULATION",
        "acceptance_incomplete": True,
    }
a.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
print(
    json.dumps(
        {k: v for k, v in result.items() if k not in ["visits", "loading_inspection"]}
    )
)
raise SystemExit(0 if result["status"] == "PASS" else 1)
