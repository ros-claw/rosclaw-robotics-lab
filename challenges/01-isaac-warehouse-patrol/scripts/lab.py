#!/usr/bin/env python3
"""One entry point and append-only, independent-reset Native regression ledger."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]


def run_script(name, *args, **kwargs):
    return subprocess.run([str(ROOT / "scripts" / name), *map(str, args)], cwd=ROOT, check=True, **kwargs)


def git_sha(path):
    return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()


def regression(destination):
    # Linux pathname Unix sockets allow at most 107 encoded bytes. Check
    # the longest Native operator socket before starting a GPU environment.
    socket_path = destination / "01-unmapped-box/native/home/run/operator.sock"
    if len(os.fsencode(socket_path)) > 107:
        raise ValueError("Native Unix socket path exceeds 107 bytes; choose a shorter result directory")
    destination.mkdir(parents=True, exist_ok=False)
    upstream = Path(os.environ.get("ROSCLAW_SOURCE", Path.home() / "sim/rosclaw-upstream"))
    python = upstream / ".venv/bin/python"
    source_sha = git_sha(REPO)
    if subprocess.check_output(["git", "-C", str(REPO), "status", "--porcelain"], text=True).strip():
        raise RuntimeError("Freeze a clean source commit before regression")
    image = os.environ.get("ROS_CONTAINER_IMAGE", "rosclaw/warehouse-jazzy:6.1.0-public")
    image_id = subprocess.check_output(["docker", "image", "inspect", image, "--format", "{{.Id}}"], text=True).strip()
    files = list((ROOT / "config").glob("*")) + list((ROOT / "evaluator").glob("*.py"))
    freeze = {"lab_commit": source_sha, "rosclaw_commit": git_sha(upstream), "docker_image_id": image_id,
              "configuration_and_evaluator_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()},
              "plan": "Exactly 15 attempts; 5 independent resets per condition; all attempts retained; no success selection",
              "capture_profile": "follow 1280x720, capture disabled, one viewport; separate from historical three-view video"}
    (destination / "freeze.json").write_text(json.dumps(freeze, indent=2) + "\n")
    conditions = [("standard", ["entry", "shelf", "aisle", "home"], False),
                  ("reordered", ["aisle", "entry", "shelf", "home"], False),
                  ("unmapped-box", ["aisle", "entry", "shelf", "home"], True)]
    for repetition in range(1, 6):
        for condition, order, obstacle in conditions:
            if git_sha(REPO) != source_sha or git_sha(upstream) != freeze["rosclaw_commit"]:
                raise RuntimeError("Pinned source changed during regression")
            ident = f"{repetition:02d}-{condition}"
            attempt = destination / ident
            attempt.mkdir()
            record = {"attempt": ident, "condition": condition, "repetition": repetition, "started_wall_time": time.time(), "status": "RUNNING", "manual_interventions": [], "source_commit": source_sha}
            (destination / "current.json").write_text(json.dumps(record, indent=2))
            (attempt / "start.json").write_text(json.dumps(record, indent=2))
            env = dict(os.environ, ROSCLAW_CAMERA_VIEW="follow", ROSCLAW_CAMERA_RESOLUTION="1280x720", ROSCLAW_CAPTURE_SECONDS="0", ROSCLAW_MULTIVIEW="0", ROSCLAW_OBSTACLE_TEST=str(int(obstacle)), ROSCLAW_REQUIRE_OBSTACLE_EVIDENCE=str(int(obstacle)))
            observers = []
            try:
                with (attempt / "orchestration.log").open("w") as log:
                    run_script("demo.sh", "headless", "patrol", env=env, stdout=log, stderr=subprocess.STDOUT, timeout=1400)
                    physics = max((ROOT / "reports/runs").iterdir(), key=lambda p: p.stat().st_mtime)
                    record["physics_directory"] = str(physics)
                    observers.append(subprocess.Popen([str(ROOT / "scripts/start-agent-observers.sh")], cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT))
                    if obstacle:
                        observers.append(subprocess.Popen([str(ROOT / "scripts/ros-container.sh"), "python3", "/lab/ros2/obstacle_witness.py", "--output", f"/lab/reports/runs/{physics.name}/path-witness.jsonl"], cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT))
                    deadline = time.monotonic() + 90
                    while True:
                        try:
                            with socket.create_connection(("127.0.0.1", 19091), timeout=1):
                                break
                        except OSError:
                            if time.monotonic() > deadline:
                                raise TimeoutError("rosbridge did not become ready")
                            time.sleep(1)
                    time.sleep(3)
                    names = {"entry": "入口", "shelf": "货架区", "aisle": "仓库通道", "home": "Home"}
                    task = "请先检查机器人状态，再依次巡检" + "、".join(names[s] for s in order) + "。每站验证到达并停留，完成后保存巡检报告和记忆。"
                    if obstacle:
                        task += "遇到未标在地图上的箱体，使用实际激光数据避障。"
                    record["task"] = task
                    task_result = subprocess.run([str(ROOT / "scripts/run-native-acceptance.sh"), str(attempt / "native"), str(physics), task, *order], cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=1500)
                    record["native_exit_code"] = task_result.returncode
                    time.sleep(3)
                    evaluation = subprocess.run([str(python), str(ROOT / "evaluator/run_report.py"), "--directory", str(attempt / "native"), "--output", str(attempt / "acceptance.json")], cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=90)
                    if (attempt / "acceptance.json").exists():
                        record["status"] = json.loads((attempt / "acceptance.json").read_text()).get("status", "FAIL")
                    else:
                        record["status"] = "INCOMPLETE"
                    record["evaluator_exit_code"] = evaluation.returncode
                    if task_result.returncode != 0 and record["status"] == "PASS":
                        record["status"] = "FAIL"
            except KeyboardInterrupt:
                record.update(status="INTERRUPTED", error="Operator interrupted supervisor", manual_interventions=["supervisor SIGINT; owned environment cleanup required"] )
                raise
            except Exception as exc:
                record.update(status="INCOMPLETE", error=repr(exc))
            finally:
                for child in observers:
                    child.terminate()
                try:
                    run_script("stop.sh", timeout=90, capture_output=True, text=True)
                except Exception as exc:
                    record.update(status="FAIL", cleanup_error=repr(exc))
                for child in observers:
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
                record["finished_wall_time"] = time.time()
                (attempt / "result.json").write_text(json.dumps(record, indent=2) + "\n")
                with (destination / "attempts.jsonl").open("a") as stream:
                    stream.write(json.dumps(record) + "\n")
                (destination / "current.json").write_text(json.dumps(record, indent=2))
                print(json.dumps(record), flush=True)
                if "cleanup_error" in record:
                    raise RuntimeError("Cleanup failed; do not overlap physical sessions")
    rows = [json.loads(line) for line in (destination / "attempts.jsonl").read_text().splitlines()]
    summary = {"status": "PASS" if len(rows) == 15 and all(row["status"] == "PASS" for row in rows) else "FAIL", "attempts": len(rows), "passes": sum(row["status"] == "PASS" for row in rows), "failures_and_incomplete": [row["attempt"] for row in rows if row["status"] != "PASS"], "source_commit": source_sha}
    (destination / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["doctor", "start", "task", "results", "stop", "regression"])
    parser.add_argument("arguments", nargs="*")
    args = parser.parse_args()
    if args.command == "regression":
        regression(Path(args.arguments[0]).resolve())
    elif args.command == "results":
        directory = Path(args.arguments[0])
        print((directory / "summary.json" if (directory / "summary.json").exists() else directory / "current.json").read_text())
    else:
        script = {"doctor": "doctor.sh", "start": "demo.sh", "task": "run-native-acceptance.sh", "stop": "stop.sh"}[args.command]
        run_script(script, *args.arguments)


if __name__ == "__main__":
    main()
