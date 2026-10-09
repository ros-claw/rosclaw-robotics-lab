"""Reproducible actual-SIM fault supplement; distinct from full Native missions."""

import hashlib, json, math, os, signal, socket, subprocess, sys, threading, time
from pathlib import Path
import argparse

parser = argparse.ArgumentParser(
    description="Actual isolated-SIM adapter fault supplement; never a Native/hardware acceptance"
)
parser.add_argument(
    "--config",
    type=Path,
    required=True,
    help="execution_config.json from your own accepted Native SIM run",
)
parser.add_argument(
    "--output",
    type=Path,
    required=True,
    help="new output directory outside Git; never overwritten",
)
parser.add_argument(
    "--fault",
    choices=["timeout", "disconnect", "pause", "navigator-abort"],
    action="append",
    required=True,
)
parser.add_argument(
    "--isolated-simulation",
    action="store_true",
    required=True,
    help="confirm dedicated idle lab SIM environment",
)
args = parser.parse_args()
if len(args.fault) != len(set(args.fault)):
    parser.error("Each fault may be requested only once per output directory")
ROOT = Path(__file__).resolve().parents[2]
FIX = Path(__file__).resolve().parent
PYTHON = Path(sys.executable)
OUT = args.output.resolve()
if (ROOT / ".runtime/sim-process.json").exists():
    parser.error(
        "Stop your owned lab session before starting an independent fault reset"
    )
if OUT.is_relative_to(ROOT.parents[1]):
    parser.error("Keep evidence output outside the source repository")
CONFIG = json.loads(args.config.read_text())
if (
    CONFIG.get("body_id") != "nova_carter_isaac"
    or CONFIG.get("physics_body_path") != "/World/Nova_Carter_ROS/chassis_link"
):
    parser.error("Only the documented Nova Carter SIM config is supported")
if "body_snapshot_hash" not in CONFIG:
    parser.error("Missing immutable Body binding")
OUT.mkdir(parents=True, exist_ok=False)
sys.path.insert(0, str(ROOT / "rosclaw"))
sys.path.insert(0, str(ROOT / "evaluator"))
from point_executor import PointExecutor, ScanWitness
from rosclaw.connectors.ros.action_client import Ros2ActionClient
from rosclaw.connectors.ros.transport.base import RosbridgeEndpoint
from rosclaw.connectors.ros.transport.rosbridge import RosbridgeTransport
from rosclaw.kernel import ExecutionMode
from types import SimpleNamespace
from datetime import datetime, timedelta, timezone

for fault in args.fault:
    run = OUT / fault
    run.mkdir()
    env = dict(
        os.environ,
        ROSCLAW_CAPTURE_SECONDS="0",
        ROSCLAW_MULTIVIEW="0",
        ROSCLAW_OBSTACLE_TEST="0",
        ROSCLAW_CAMERA_RESOLUTION="1280x720",
        ROSCLAW_CAMERA_VIEW="follow",
    )
    children = []
    transports = []
    injector = None
    injection_events = []
    try:
        with (run / "orchestration.log").open("w") as log:
            subprocess.run(
                [str(ROOT / "scripts/demo.sh"), "headless", "patrol"],
                env=env,
                cwd=ROOT,
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
            time.sleep(3)
            config = dict(CONFIG)
            config["physics_directory"] = str(physics)
            (run / "execution_config.json").write_text(json.dumps(config, indent=2))
            if fault in ["timeout", "disconnect"]:
                subprocess.run(
                    [
                        str(PYTHON),
                        str(ROOT / "rosclaw/fault_probe.py"),
                        "--config",
                        str(run / "execution_config.json"),
                        "--directory",
                        str(run / "adapter"),
                        "--fault",
                        fault,
                        "--site",
                        "entry",
                    ],
                    env={**env, "PYTHONPATH": str(ROOT / "rosclaw")},
                    cwd=ROOT,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    check=True,
                    timeout=90,
                )
                summary = json.loads((run / "adapter/summary.json").read_text())
            else:
                transports = [
                    RosbridgeTransport(
                        RosbridgeEndpoint.from_url("ws://127.0.0.1:19091")
                    )
                    for _ in range(2)
                ]
                for transport in transports:
                    if not transport.connect().ok:
                        raise RuntimeError("transport connect")

                class TestBTClient(Ros2ActionClient):
                    def send_goal(self, **kwargs):
                        kwargs["args"] = {
                            **kwargs["args"],
                            "behavior_tree": f"/lab/reports/runs/{physics.name}/abort_with_active_controller.xml",
                        }
                        return super().send_goal(**kwargs)

                if fault == "navigator-abort":
                    for name in [
                        "delayed_follow_path.py",
                        "abort_with_active_controller.xml",
                    ]:
                        (physics / name).write_bytes((FIX / name).read_bytes())
                    injector = subprocess.Popen(
                        [
                            str(ROOT / "scripts/ros-container.sh"),
                            "python3",
                            f"/lab/reports/runs/{physics.name}/delayed_follow_path.py",
                            "--output",
                            f"/lab/reports/runs/{physics.name}/abort-status.jsonl",
                            "--physics",
                            f"/lab/reports/runs/{physics.name}/physics-latest.json",
                            "--isolated-simulation-fault-test",
                        ],
                        env={
                            **env,
                            "ROSCLAW_CONTAINER_NAME": "rosclaw-warehouse-abort-injector",
                        },
                        cwd=ROOT,
                        stdout=log,
                        stderr=subprocess.STDOUT,
                    )
                    children.append(injector)
                    time.sleep(5)
                    client = TestBTClient(transports[0])
                else:
                    client = Ros2ActionClient(transports[0])
                sensor = ScanWitness(transports[1])
                time.sleep(2)
                executor = PointExecutor(
                    root=run, config=config, client=client, sensor=sensor
                )
                if fault == "pause":
                    saved = json.loads((ROOT / ".runtime/sim-process.json").read_text())

                    def pause_after_motion():
                        deadline = time.monotonic() + 20
                        while time.monotonic() < deadline:
                            sample = json.loads(
                                (physics / "physics-latest.json").read_text()
                            )
                            if math.hypot(*sample["linear_velocity_xyz"][:2]) > 0.1:
                                break
                            time.sleep(0.1)
                        else:
                            injection_events.append({"event": "NO_MOTION_BEFORE_FAULT"})
                            return
                        proc = Path(f"/proc/{saved['pid']}/stat")
                        if (
                            proc.read_text().rsplit(")", 1)[1].split()[19]
                            != saved["birth_ticks"]
                        ):
                            return
                        injection_events.append(
                            {
                                "wall_time": time.time(),
                                "event": "SIGSTOP_owned_simulator",
                                "sim_time": sample["sim_time"],
                                "actual_speed_mps": math.hypot(
                                    *sample["linear_velocity_xyz"][:2]
                                ),
                            }
                        )
                        os.kill(saved["pid"], signal.SIGSTOP)
                        try:
                            time.sleep(4)
                            paused = json.loads(
                                (physics / "physics-latest.json").read_text()
                            )
                            injection_events.append(
                                {
                                    "wall_time": time.time(),
                                    "event": "paused_snapshot_observed",
                                    "sim_time": paused["sim_time"],
                                    "snapshot_wall_time": paused["wall_time"],
                                }
                            )
                        finally:
                            if (
                                proc.exists()
                                and proc.read_text().rsplit(")", 1)[1].split()[19]
                                == saved["birth_ticks"]
                            ):
                                os.kill(saved["pid"], signal.SIGCONT)
                                injection_events.append(
                                    {
                                        "wall_time": time.time(),
                                        "event": "SIGCONT_owned_simulator",
                                    }
                                )

                    pause_thread = threading.Thread(target=pause_after_motion)
                    pause_thread.start()
                action = SimpleNamespace(
                    execution_mode=ExecutionMode.SIMULATION,
                    body_id=config["body_id"],
                    body_snapshot_hash=config["body_snapshot_hash"],
                    capability_id="navigation.navigate_to_pose",
                    action_id="v02-fault-" + fault,
                    arguments={"site_id": "entry"},
                    deadline_at=datetime.now(timezone.utc) + timedelta(seconds=45),
                )
                started = time.time()
                result = executor(action)
                if fault == "pause":
                    pause_thread.join(timeout=10)
                sensor.close()
                client.close()
                evidence = json.loads(
                    next((run / "actions").glob("*.json")).read_text()
                )
                fallback = [
                    e
                    for e in evidence.get("events", [])
                    if e["event"] == "DDS_cancel_fallback"
                ]
                ack = any(
                    e["returncode"] == 0 and json.loads(e["stdout"]).get("acknowledged")
                    for e in fallback
                )
                summary = {
                    "fault": fault,
                    "scope": "actual SIM adapter negative test, not Native mission",
                    "final_state": result.final_state.value,
                    "physical_stop_verified": evidence.get(
                        "physical_stop_verified", False
                    ),
                    "error": evidence.get("error"),
                    "nav2": evidence.get("nav2"),
                    "wall_seconds": time.time() - started,
                    "DDS_fallback_events": fallback,
                    "injection_events": injection_events,
                    "cancel_confirmed": evidence.get("nav2", {}).get("status") == 5
                    or ack,
                }
                if fault == "navigator-abort":
                    rows = (
                        [
                            json.loads(line)
                            for line in (physics / "abort-status.jsonl")
                            .read_text()
                            .splitlines()
                        ]
                        if (physics / "abort-status.jsonl").exists()
                        else []
                    )
                    actual_overlap = []
                    for row in rows:
                        statuses = row.get("all_servers", {})
                        navigator = statuses.get("/navigate_to_pose", {})
                        controller = statuses.get("/follow_path", {})
                        if (
                            any(g["status"] == 6 for g in navigator.get("goals", []))
                            and any(
                                g["status"] in [1, 2]
                                for g in controller.get("goals", [])
                            )
                            and abs(
                                navigator.get("wall_time", 0)
                                - controller.get("wall_time", 0)
                            )
                            < 2
                        ):
                            actual_overlap.append(row)
                    summary["actual_navigator_abort_with_active_controller"] = bool(
                        actual_overlap
                    )
                    summary["overlap_samples"] = actual_overlap
                    summary["fault_source_sha256"] = {
                        name: hashlib.sha256((FIX / name).read_bytes()).hexdigest()
                        for name in [
                            "delayed_follow_path.py",
                            "abort_with_active_controller.xml",
                        ]
                    }
                    summary["nav2_aborted"] = (
                        evidence.get("nav2", {}).get("status") == 6
                    )
                    summary["fault_injected"] = (
                        bool(actual_overlap) and summary["nav2_aborted"]
                    )
                else:
                    summary["fault_injected"] = any(
                        e["event"] == "SIGSTOP_owned_simulator"
                        for e in injection_events
                    )
                if fault == "pause":
                    post = []
                    stable = []
                    post_deadline = time.monotonic() + 8
                    while time.monotonic() < post_deadline:
                        sample = json.loads(
                            (physics / "physics-latest.json").read_text()
                        )
                        if (
                            0 <= time.time() - sample["wall_time"] < 2
                            and sample["timeline_playing"]
                            and sample["collision_observer_complete"]
                            and sample["collision_count"] == 0
                        ):
                            if not post or post[-1]["sequence"] != sample["sequence"]:
                                post.append(sample)
                                if (
                                    math.hypot(*sample["linear_velocity_xyz"][:2])
                                    <= 0.02
                                    and abs(sample["angular_velocity_xyz"][2]) <= 0.1
                                ):
                                    stable.append(sample)
                                else:
                                    stable = []
                            if (
                                len(stable) > 2
                                and stable[-1]["sim_time"] - stable[0]["sim_time"]
                                >= 0.5
                            ):
                                break
                        time.sleep(0.1)
                    summary["post_resume_observations"] = post
                    summary["post_resume_stop_verified"] = (
                        len(stable) > 2
                        and stable[-1]["sim_time"] - stable[0]["sim_time"] >= 0.5
                    )
                    summary["post_resume_stable_stop_observations"] = stable
                    summary["status"] = (
                        "PASS"
                        if summary["final_state"] == "FAILED"
                        and summary["cancel_confirmed"]
                        and summary["fault_injected"]
                        and summary["post_resume_stop_verified"]
                        else "FAIL"
                    )
                    summary["physical_claim_boundary"] = (
                        "Initial stale/paused physics cannot verify stop; post-resume advancing PhysX samples separately verify stopped state."
                    )
                else:
                    summary["status"] = (
                        "PASS"
                        if summary["final_state"] == "FAILED"
                        and summary["physical_stop_verified"]
                        and summary["cancel_confirmed"]
                        and summary["fault_injected"]
                        else "FAIL"
                    )
            summary["physics_directory"] = str(physics)
            (run / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            print(
                json.dumps(
                    {
                        "fault": fault,
                        "status": summary["status"],
                        "physical_stop_verified": summary["physical_stop_verified"],
                    }
                ),
                flush=True,
            )
    except Exception as exc:
        (run / "summary.json").write_text(
            json.dumps(
                {"fault": fault, "status": "INCOMPLETE", "error": repr(exc)}, indent=2
            )
        )
        print(fault, repr(exc), flush=True)
    finally:
        for transport in transports:
            transport.close()
        for child in children:
            child.terminate()
        subprocess.run(
            [str(ROOT / "scripts/stop.sh")], cwd=ROOT, env=env, check=True, timeout=90
        )
        for child in children:
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
summary = {
    f.name: json.loads((f / "summary.json").read_text())["status"]
    for f in OUT.iterdir()
    if f.is_dir()
}
(OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
raise SystemExit(0 if all(value == "PASS" for value in summary.values()) else 1)
