#!/usr/bin/env python3
"""Isolated-SIM single-input pilot; preserves failures and always cleans owned resources."""

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

HERE = Path(__file__).resolve().parent
PATROL = HERE.parent / "01-isaac-warehouse-patrol"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--isolated-simulation", action="store_true", required=True)
    p.add_argument("--require-target-lidar", action="store_true")
    p.add_argument("--fixture", type=Path, required=True)
    p.add_argument("--thresholds", type=Path)
    p.add_argument("--authorized-observation-bounds", type=Path)
    p.add_argument("--record-multiview", action="store_true")
    p.add_argument(
        "--task",
        default="请检查本次任务起点西侧最近的一组开发集货架：选择面向货架的安全观察位置，到达并停留，然后返回本次任务的实际起点。最后保存验证报告和记忆。",
    )
    a = p.parse_args()
    output = a.output.resolve()
    if (PATROL / ".runtime/sim-process.json").exists():
        raise RuntimeError("Stop the owned lab session before independent reset")
    if output.exists():
        raise ValueError("Existing output refused; retain prior attempts")
    if len(os.fsencode(output / "native/home/run/operator.sock")) > 107:
        raise ValueError("Use shorter output path")
    if not a.fixture.is_file():
        raise ValueError("Operator reset fixture required")
    source = os.environ["ROSCLAW_SOURCE"]
    python = str(Path(source) / ".venv/bin/python")
    output.mkdir(parents=True)
    observers = None
    native = None
    record = {
        "status": "RUNNING",
        "started_wall_time": time.time(),
        "task": __import__("prepare_loading").TASK,
        "fixture_sha256": __import__("hashlib")
        .sha256(a.fixture.read_bytes())
        .hexdigest(),
        "fixture": str(a.fixture.resolve()),
        "manual_interventions": [],
        "require_target_lidar": a.require_target_lidar,
        "kind": "loading-area engineering attempt; no held-out or fairness claim",
    }
    (output / "start.json").write_text(json.dumps(record, indent=2) + "\n")
    env = dict(
        os.environ,
        ROSCLAW_CAMERA_VIEW="follow",
        ROSCLAW_CAMERA_RESOLUTION="1920x1080" if a.record_multiview else "1280x720",
        ROSCLAW_CAPTURE_SECONDS="1600" if a.record_multiview else "0",
        ROSCLAW_MULTIVIEW="1" if a.record_multiview else "0",
        ROSCLAW_OBSTACLE_TEST="0",
        ROSCLAW_LOADING_AUDIT="1",
        ROSCLAW_FORK_CONTROL="0",
        ROSCLAW_LOADING_FIXTURE=str(a.fixture.resolve()),
    )

    def run(command, **kw):
        return subprocess.run(command, env=env, check=True, **kw)

    try:
        with (output / "orchestration.log").open("w") as log:
            run(
                [str(PATROL / "scripts/demo.sh"), "headless", "patrol"],
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=1400,
            )
            physics = max(
                (PATROL / "reports/runs").iterdir(), key=lambda p: p.stat().st_mtime
            )
            record["physics_directory"] = str(physics)
            observers = subprocess.Popen(
                [str(PATROL / "scripts/start-agent-observers.sh")],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            deadline = time.monotonic() + 90
            while True:
                try:
                    with socket.create_connection(("127.0.0.1", 19091), timeout=1):
                        break
                except OSError:
                    if time.monotonic() > deadline:
                        raise TimeoutError("Observation transport not ready")
                    time.sleep(1)
            time.sleep(3)
            command = [
                python,
                str(HERE / "prepare_loading.py"),
                "--directory",
                str(output / "native"),
                "--physics",
                str(physics),
                "--map-yaml",
                str(
                    Path(os.environ["ISAAC_ROS_WS"])
                    / "src/navigation/carter_navigation/maps/carter_warehouse_navigation.yaml"
                ),
            ]
            if a.thresholds:
                command.extend(["--thresholds", str(a.thresholds.resolve())])
            if a.authorized_observation_bounds:
                command.extend(
                    [
                        "--authorized-observation-bounds",
                        str(a.authorized_observation_bounds.resolve()),
                    ]
                )
            run(command, stdout=log, stderr=subprocess.STDOUT, timeout=30)
            record["native_started_wall_time"] = time.time()
            native = subprocess.Popen(
                [
                    python,
                    str(PATROL / "rosclaw/native.py"),
                    "--directory",
                    str(output / "native"),
                ],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                native.wait(timeout=1000)
            except subprocess.TimeoutExpired:
                record["native_timeout"] = True
                native.send_signal(signal.SIGINT)
                try:
                    native.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    os.killpg(native.pid, signal.SIGKILL)
                    native.wait()
            record["native_exit_code"] = native.returncode
            record["native_finished_wall_time"] = time.time()
            if a.record_multiview:
                # Let the already scheduled capture complete after Native exits.
                time.sleep(3)
                from recording import verify_recording

                coverage = verify_recording(
                    physics / "baseline-frames",
                    record["native_started_wall_time"],
                    record["native_finished_wall_time"],
                )
                (output / "recording-acceptance.json").write_text(
                    json.dumps(coverage, indent=2) + "\n"
                )
                record["recording_status"] = coverage["status"]
            evaluation = subprocess.run(
                [
                    python,
                    str(HERE / "evaluate_loading.py"),
                    "--directory",
                    str(output / "native"),
                    "--output",
                    str(output / "acceptance.json"),
                ],
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                timeout=90,
            )
            record["evaluator_exit_code"] = evaluation.returncode
            record["status"] = json.loads((output / "acceptance.json").read_text())[
                "status"
            ]
            if native.returncode != 0:
                record["status"] = "FAIL"
    except BaseException as exc:
        record.update(
            status="INTERRUPTED"
            if isinstance(exc, KeyboardInterrupt)
            else "INCOMPLETE",
            error=repr(exc),
        )
        if isinstance(exc, KeyboardInterrupt):
            record["manual_interventions"].append("Supervisor interrupted")
    finally:
        if native and native.poll() is None:
            native.send_signal(signal.SIGINT)
            try:
                native.wait(timeout=45)
            except subprocess.TimeoutExpired:
                os.killpg(native.pid, signal.SIGKILL)
                native.wait()
        if observers:
            observers.terminate()
            try:
                observers.wait(timeout=5)
            except subprocess.TimeoutExpired:
                observers.kill()
                observers.wait()
        cleanup = subprocess.run(
            [str(PATROL / "scripts/stop.sh")],
            env=env,
            capture_output=True,
            text=True,
            timeout=45,
        )
        record.update(
            finished_wall_time=time.time(), cleanup_exit_code=cleanup.returncode
        )
        (output / "result.json").write_text(json.dumps(record, indent=2) + "\n")
        print(json.dumps(record, ensure_ascii=False))
    raise SystemExit(
        0
        if record["status"] == "PASS"
        and record["cleanup_exit_code"] == 0
        and record.get("recording_status", "PASS") == "PASS"
        else 1
    )


if __name__ == "__main__":
    main()
