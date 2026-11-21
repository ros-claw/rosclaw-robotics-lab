"""Start, execute, and clean up one protected adaptive navigation mission."""

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

LAB = Path(__file__).with_name("lab.py")


def mission_status(code, terminal):
    if code:
        return "failed", "controller_exit"
    if terminal.get("event") == "route_completed":
        return "succeeded", None
    if terminal.get("event") == "hold":
        return "hold", terminal["reason"]
    return "failed", "missing_terminal_evidence"


def run_mission(args):
    if not os.environ.get("OPENAI_API_KEY"):
        raise RuntimeError("OPENAI_API_KEY is required")
    launch = [sys.executable, str(LAB), "start", "--protected", "--rgbd",
              "--layout", str(args.layout.resolve()), "--domain", str(args.domain),
              "--drive-profile", args.drive_profile, "--capture-mode", args.capture_mode]
    if args.scenario_obstacles:
        launch += ["--scenario-obstacles", str(args.scenario_obstacles.resolve())]
    if args.record_overview:
        launch.append("--record-overview")
    sim_env = {key: value for key, value in os.environ.items()
               if key not in {"OPENAI_API_KEY", "OPENAI_BASE_URL"}}

    run = None
    code = 1
    process = subprocess.Popen(launch, stdout=subprocess.PIPE, text=True, env=sim_env)
    try:
        for line in process.stdout:
            print(line, end="", flush=True)
            if line.startswith("READY "):
                run = Path(line.removeprefix("READY ").strip())
                command = [sys.executable, str(LAB), "luna-run", "--expected-run", str(run),
                           "--local-retries", str(args.local_retries),
                           "--max-observations", str(args.max_observations),
                           "--mission-time", str(args.mission_time)]
                if args.vision_interval is not None:
                    command += ["--vision-interval", str(args.vision_interval),
                                "--vision-budget", str(args.vision_budget), "--vision-profile", args.vision_profile]
                if args.mission_policy:
                    command += ["--mission-policy", str(args.mission_policy.resolve())]
                code = subprocess.run(command).returncode
                break
    finally:
        if process.poll() is None:
            process.terminate()
        process.wait(timeout=45)
        process.stdout.close()
    if run is None:
        raise RuntimeError("Isaac/Nav2 did not reach readiness")
    sessions = list(run.glob("luna-session-*.jsonl"))
    events = sessions[0].read_text().splitlines() if len(sessions) == 1 else []
    status, reason = mission_status(code, json.loads(events[-1]) if events else {})
    result = {"status": status, "reason": reason, "return_code": code, "run": str(run)}
    (run / "mission-summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return {"succeeded": 0, "hold": 3, "failed": 1}[status]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=189)
    parser.add_argument("--record-overview", action="store_true")
    parser.add_argument("--scenario-obstacles", type=Path)
    parser.add_argument("--mission-policy", type=Path)
    parser.add_argument("--local-retries", type=int, choices=range(6), default=2)
    parser.add_argument("--max-observations", type=int, choices=range(1, 6), default=3)
    parser.add_argument("--mission-time", type=float, default=150)
    parser.add_argument("--vision-interval", type=float)
    parser.add_argument("--vision-budget", type=int, default=7)
    parser.add_argument("--vision-profile", choices=("standard", "lean"), default="lean")
    parser.add_argument("--drive-profile", choices=("baseline", "quicker"), default="baseline")
    parser.add_argument("--capture-mode", choices=("persistent", "oneshot"), default="persistent")
    args = parser.parse_args()
    try:
        return run_mission(args)
    except (RuntimeError, OSError, subprocess.TimeoutExpired) as error:
        print(error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
