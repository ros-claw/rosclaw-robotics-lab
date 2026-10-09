"""Export only actual SDK user inputs and tool names; retain private journals locally."""

import argparse
import hashlib
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--attempts", nargs="+", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    attempts = []
    for attempt in a.attempts:
        root = attempt / "native"
        task = json.loads((root / "execution_config.json").read_text())["task"]
        users, tools, hashes = [], [], []
        for session in sorted((root / "home/agent/sessions").glob("*.jsonl")):
            hashes.append(hashlib.sha256(session.read_bytes()).hexdigest())
            for line in session.read_text().splitlines():
                row = json.loads(line)
                message = row.get("message", {})
                content = message.get("content", [])
                if message.get("role") == "user":
                    value = (
                        content
                        if isinstance(content, str)
                        else "".join(
                            item.get("text", "")
                            for item in content
                            if isinstance(item, dict) and item.get("type") == "text"
                        )
                    )
                    # Do not export an unexpected private prompt.
                    if value != task:
                        raise ValueError(
                            "User content differs from the declared public task"
                        )
                    users.append({"timestamp": row["timestamp"], "text": task})
                if message.get("role") == "assistant" and isinstance(content, list):
                    for item in content:
                        if item.get("type") == "toolCall":
                            tools.append(
                                {"timestamp": row["timestamp"], "name": item["name"]}
                            )
        if not hashes:
            raise ValueError("No actual SDK journal")
        attempts.append(
            {
                "attempt": attempt.name,
                "task_input_count": len(users),
                "task_inputs": users,
                "tool_calls": tools,
                "private_journal_sha256": hashes,
                "status": "PASS" if len(users) == 1 else "FAIL",
            }
        )
    result = {
        "status": "PASS" if all(x["status"] == "PASS" for x in attempts) else "FAIL",
        "scope": "Sanitized extract from actual local SDK journals, not an independent attestation. Operator SIM decisions are separate from task inputs. Assistant prose/thinking, tool arguments/results and authentication are excluded.",
        "attempts": attempts,
    }
    a.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"status": result["status"], "attempts": len(attempts)}))


if __name__ == "__main__":
    main()
