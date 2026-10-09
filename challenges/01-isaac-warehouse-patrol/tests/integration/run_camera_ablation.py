"""Exploratory paired camera-profile ablation, separate from Native acceptance."""

import hashlib, json, os, socket, subprocess, sys, time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace
import argparse

parser = argparse.ArgumentParser(
    description="Exploratory actual-SIM camera ablation; no model or Native task"
)
parser.add_argument("--config", type=Path, required=True)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--isolated-simulation", action="store_true", required=True)
args = parser.parse_args()
ROOT = Path(__file__).resolve().parents[2]
OUT = args.output.resolve()
if (ROOT / ".runtime/sim-process.json").exists():
    parser.error("Stop the owned lab session before camera ablation")
if OUT.is_relative_to(ROOT.parents[1]):
    parser.error("Keep results outside Git")
CONFIG = json.loads(args.config.read_text())
if (
    CONFIG.get("body_id") != "nova_carter_isaac"
    or CONFIG.get("physics_body_path") != "/World/Nova_Carter_ROS/chassis_link"
):
    parser.error("Only the documented Nova Carter SIM config is supported")
if "body_snapshot_hash" not in CONFIG:
    parser.error("Missing immutable Body binding")
sys.path[:0] = [str(ROOT / "rosclaw"), str(ROOT / "evaluator")]
from point_executor import PointExecutor, ScanWitness
from rosclaw.connectors.ros.action_client import Ros2ActionClient
from rosclaw.connectors.ros.transport.base import RosbridgeEndpoint
from rosclaw.connectors.ros.transport.rosbridge import RosbridgeTransport
from rosclaw.kernel import ExecutionMode

OUT.mkdir(parents=True, exist_ok=False)
profiles = [
    ("base", {"ROSCLAW_MULTIVIEW": "0", "ROSCLAW_CAPTURE_SECONDS": "0"}),
    ("single-png", {"ROSCLAW_MULTIVIEW": "0", "ROSCLAW_CAPTURE_SECONDS": "1200"}),
    ("three-png", {"ROSCLAW_MULTIVIEW": "1", "ROSCLAW_CAPTURE_SECONDS": "1200"}),
]
rows = []
for repetition in [1, 2]:
    # Reverse profile order in repeat 2 to reduce drift/cache order confounding.
    for profile, settings in profiles if repetition == 1 else list(reversed(profiles)):
        run = OUT / f"{repetition:02d}-{profile}"
        run.mkdir()
        children = []
        transports = []
        env = dict(
            os.environ,
            ROSCLAW_CAMERA_VIEW="follow",
            ROSCLAW_CAMERA_RESOLUTION="1920x1080",
            ROSCLAW_OBSTACLE_TEST="0",
            **settings,
        )
        summary = {
            "profile": profile,
            "repetition": repetition,
            "scope": "SIM single-site adapter camera ablation; no model or Native mission",
            "status": "INCOMPLETE",
            "benchmark_source_sha256": hashlib.sha256(
                Path(__file__).read_bytes()
            ).hexdigest(),
            "source_commit": subprocess.check_output(
                ["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True
            ).strip(),
            "started_wall_time": time.time(),
        }

        def cpu(pid):
            s = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
            return (int(s[11]) + int(s[12])) / os.sysconf("SC_CLK_TCK")

        try:
            with (run / "orchestration.log").open("w") as log:
                subprocess.run(
                    [str(ROOT / "scripts/demo.sh"), "headless", "patrol"],
                    cwd=ROOT,
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    check=True,
                    timeout=1400,
                )
                physics = max(
                    (ROOT / "reports/runs").iterdir(), key=lambda p: p.stat().st_mtime
                )
                children.append(
                    subprocess.Popen(
                        [str(ROOT / "scripts/start-agent-observers.sh")],
                        cwd=ROOT,
                        env=env,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                    )
                )
                deadline = time.monotonic() + 90
                while True:
                    try:
                        with socket.create_connection(("127.0.0.1", 19091), timeout=1):
                            break
                    except OSError:
                        if time.monotonic() > deadline:
                            raise TimeoutError("bridge readiness")
                        time.sleep(1)
                time.sleep(5)
                config = dict(CONFIG)
                config["physics_directory"] = str(physics)
                (run / "execution_config.json").write_text(json.dumps(config, indent=2))
                transports = [
                    RosbridgeTransport(
                        RosbridgeEndpoint.from_url("ws://127.0.0.1:19091")
                    )
                    for _ in range(2)
                ]
                for t in transports:
                    if not t.connect().ok:
                        raise RuntimeError("connect")
                client = Ros2ActionClient(transports[0])
                sensor = ScanWitness(transports[1])
                deadline = time.monotonic() + 10
                while not sensor.snapshot() and time.monotonic() < deadline:
                    time.sleep(0.1)
                adapter = PointExecutor(
                    root=run, config=config, client=client, sensor=sensor
                )
                action = SimpleNamespace(
                    execution_mode=ExecutionMode.SIMULATION,
                    body_id=config["body_id"],
                    body_snapshot_hash=config["body_snapshot_hash"],
                    capability_id="navigation.navigate_to_pose",
                    action_id="perf-" + run.name,
                    arguments={"site_id": "entry"},
                    deadline_at=datetime.now(timezone.utc) + timedelta(seconds=420),
                )
                kit = json.loads((ROOT / ".runtime/sim-process.json").read_text())[
                    "pid"
                ]
                cpu_start = cpu(kit)
                start = time.time()
                result = adapter(action)
                end = time.time()
                cpu_end = cpu(kit)
                sensor.close()
                client.close()
                evidence_path = next((run / "actions").glob("*.json"))
                evidence = json.loads(evidence_path.read_text())
                samples = evidence.get("trajectory", [])
                summary.update(
                    status="PASS"
                    if result.final_state.value == "COMPLETED"
                    else "FAIL",
                    actual_final_state=result.final_state.value,
                    physics_directory=str(physics),
                    wall_seconds=end - start,
                    kit_cpu_seconds=cpu_end - cpu_start,
                    kit_average_cpu_cores=(cpu_end - cpu_start) / (end - start),
                    evidence_sha256=hashlib.sha256(
                        evidence_path.read_bytes()
                    ).hexdigest(),
                    body_snapshot_hash=config["body_snapshot_hash"],
                    position_error_m=evidence.get("verification", {}).get(
                        "position_error_m"
                    ),
                    collision_count=max(
                        (s["collision_count"] for s in samples), default=None
                    ),
                    error=evidence.get("error"),
                    sim_to_wall_ratio=(samples[-1]["sim_time"] - samples[0]["sim_time"])
                    / (samples[-1]["wall_time"] - samples[0]["wall_time"])
                    if len(samples) > 1
                    else None,
                    route_distance_m=sum(
                        __import__("math").dist(
                            a["physics_transforms_xyzw"][0][:2],
                            b["physics_transforms_xyzw"][0][:2],
                        )
                        for a, b in zip(samples, samples[1:])
                    ),
                )
                timestamps = physics / "baseline-frames/timestamps.jsonl"
                captures = (
                    [json.loads(line) for line in timestamps.read_text().splitlines()]
                    if timestamps.exists()
                    else []
                )
                captures = [
                    c for c in captures if start <= c.get("wall_time", 0) <= end
                ]
                summary["capture_group_count"] = len(captures)
                summary["actual_capture_groups_per_wall_second"] = len(captures) / (
                    end - start
                )
                summary["startup_wall_seconds"] = (
                    json.loads((physics / "run-ready.json").read_text())["wall_time"]
                    - json.loads((physics / "run-start.json").read_text())["wall_time"]
                )
        except Exception as exc:
            summary["exception"] = repr(exc)
        finally:
            for t in transports:
                t.close()
            for child in children:
                child.terminate()
            subprocess.run(
                [str(ROOT / "scripts/stop.sh")],
                cwd=ROOT,
                env=env,
                check=True,
                timeout=90,
            )
            for child in children:
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
            summary["finished_wall_time"] = time.time()
            (run / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            rows.append(summary)
            with (OUT / "attempts.jsonl").open("a") as stream:
                stream.write(json.dumps(summary) + "\n")
            print(json.dumps(summary), flush=True)
(OUT / "summary.json").write_text(
    json.dumps(
        {
            "status": "PASS"
            if len(rows) == 6 and all(r["status"] == "PASS" for r in rows)
            else "FAIL",
            "attempts": rows,
            "interpretation": "Two repeats per composite profile, exploratory. Base retains rendering for viewport and RTX LiDAR; no pure-physics or pure-render timing claim.",
        },
        indent=2,
    )
    + "\n"
)

raise SystemExit(
    0 if len(rows) == 6 and all(r["status"] == "PASS" for r in rows) else 1
)
