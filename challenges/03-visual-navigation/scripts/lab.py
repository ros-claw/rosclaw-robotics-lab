#!/usr/bin/env python3
"""Start and verify the Isaac 6.2 / Nav2 baseline without sending a goal."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from urllib.parse import urlsplit
from uuid import uuid4

import yaml

CHALLENGE = Path(__file__).resolve().parents[1]
REPO = CHALLENGE.parents[1]
WORKSPACE = REPO.parent
SCENE = "Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"
NVIDIA_REV = "f215318fb48913d30b938259b2163dfcf4df0caf"
ROS_SETUP = Path("/opt/ros/humble/setup.bash")
SETTINGS = yaml.safe_load((CHALLENGE / "config/visual_nav.yaml").read_text())
ISAAC = Path(os.getenv("ISAACSIM_PATH", "/data/omniverse/isaac-sim-6.2.0")).resolve()
NVIDIA = Path(os.getenv("LUNA_ISAAC_ROS_WS", WORKSPACE / "isaac-ros-6.2")).resolve()
SIMSCENE = Path(os.getenv("LUNA_SIMSCENE", WORKSPACE / "simscene")).resolve()
EVIDENCE = Path(os.getenv("LUNA_EVIDENCE", WORKSPACE / "p0-evidence")).resolve()
RUNTIME = CHALLENGE / ".runtime"


def asset_root(installation, region, explicit):
    # Host Python 3.10 lacks tomllib. Read the single installed profile setting.
    version = SETTINGS["asset_version"]
    if explicit is None:
        config = installation / "exts/isaacsim.storage.native/config/extension.toml"
        key = f'exts."isaacsim.storage.native".asset_region_profiles.{region}.asset_root'
        line = re.search(rf'^\s*{re.escape(key)}\s*=\s*("[^"]+")', config.read_text(), re.M)
        if line is None:
            raise ValueError(f"Missing NVIDIA asset profile {region}")
        explicit = json.loads(line[1])
    root = explicit.rstrip("/")
    parts = urlsplit(root)
    if (parts.username or parts.password or parts.query or parts.fragment
            or not parts.path.endswith(f"/Assets/Isaac/{version}")):
        raise ValueError(f"Expected a credential-free /Assets/Isaac/{version} root")
    if parts.scheme not in {"https", "omniverse", ""} or not parts.path.startswith("/"):
        raise ValueError("Use an HTTPS/Omniverse URL or an absolute local asset root")
    return root


def capture(command):
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=12)
        return {"exit_code": result.returncode, "stdout": result.stdout.strip(),
                "stderr": result.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"error": str(exc)}


def doctor():
    root = asset_root(ISAAC, SETTINGS["asset_region"], os.getenv("ISAACSIM_ASSET_ROOT"))
    version_file = ISAAC / "VERSION"
    bridge = ISAAC / "exts/isaacsim.ros2.bridge/config/extension.toml"
    report = {
        "repositories": {
            str(path): capture(["git", "-C", str(path), "rev-parse", "HEAD"])
            for path in (REPO, SIMSCENE, NVIDIA)
        },
        "git_status": capture(["git", "-C", str(REPO), "status", "--short"]),
        "os_release": Path("/etc/os-release").read_text(),
        "cpu": capture(["lscpu"]),
        "gpus": capture(["nvidia-smi", "--query-gpu=index,name,driver_version,memory.total",
                         "--format=csv"]),
        "host_python": sys.version,
        "isaac_python": capture([str(ISAAC / "python.sh"), "-V"]),
        "isaac_version": version_file.read_text().strip() if version_file.exists() else None,
        "bridge_version": next((line for line in bridge.read_text().splitlines()
                                if line.startswith("version")), None) if bridge.exists() else None,
        "bundled_ros": sorted(path.parent.name for path in
                              (ISAAC / "exts/isaacsim.ros2.core").glob("*/lib")),
        "host_humble_setup": ROS_SETUP.is_file(),
        "nav2_image": capture(["docker", "image", "inspect", "--format", "{{.Id}}",
                               SETTINGS["nav2_image"]]),
        "asset_root": root,
        "scene_asset": (capture(["curl", "-fsSLI", "--max-time", "10", root + "/" + SCENE])
                        if root.startswith("https:") else None),
        "credential_present": {key: bool(os.getenv(key))
                               for key in ("OPENAI_API_KEY", "OPENAI_BASE_URL")},
        "model": SETTINGS["model"],
    }
    return report


def write_json(path, data):
    draft = path.with_suffix(".tmp")
    draft.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
    draft.replace(path)


def birth_ticks(pid):
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19]
    except FileNotFoundError:
        return None


class Processes:
    def __init__(self, directory):
        self.directory = directory
        self.children = []

    def __enter__(self):
        return self

    def spawn(self, name, command, env=None):
        with (self.directory / f"{name}.log").open("ab") as log:
            process = subprocess.Popen(command, stdout=log, stderr=log,
                                       env=env, cwd=self.directory, start_new_session=True)
        self.children.append((name, process))
        return process

    def __exit__(self, *_):
        results = {}
        for name, process in reversed(self.children):
            # Signal the session's process group, including descendants of launch wrappers.
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            results[name] = {"pid": process.pid, "returncode": process.returncode}
        write_json(self.directory / "stop.json", results)


class StopRequested(BaseException):
    pass


def stop_requested(*_):
    raise StopRequested


def start(domain, gpu, timeout, rgbd, calibration, layout, protected, rmw, camera_yaw, record_overview):
    RUNTIME.mkdir(exist_ok=True)
    with (RUNTIME / "supervisor.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("This challenge already has a running supervisor") from None
        run_id = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime()) + "-" + uuid4().hex[:8]
        run = EVIDENCE / run_id
        run.mkdir(parents=True)
        write_json(RUNTIME / "active.json", {"pid": os.getpid(),
                   "birth_ticks": birth_ticks(os.getpid()), "run": str(run)})
        print(run, flush=True)
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, stop_requested)
        try:
            start_baseline(run, run_id, domain, gpu, timeout, rgbd, calibration, layout, protected, rmw, camera_yaw, record_overview)
        except StopRequested:
            write_json(run / "result.json", {"status": "stopped"})
        except (OSError, RuntimeError, ValueError) as exc:
            write_json(run / "result.json", {"status": "failed", "reason": str(exc)})
            raise
        finally:
            (RUNTIME / "active.json").unlink(missing_ok=True)


def start_baseline(run, run_id, domain, gpu, timeout, rgbd, calibration, layout, protected, rmw, camera_yaw, record_overview):
    info = doctor()
    write_json(run / "environment.json", info)
    if info["repositories"][str(NVIDIA)].get("stdout") != NVIDIA_REV:
        raise RuntimeError(f"Expected NVIDIA IsaacSim-6.2.0 checkout at {NVIDIA}")
    if not info["host_humble_setup"] or info["nav2_image"].get("exit_code") != 0:
        raise RuntimeError("Humble host setup or the challenge Nav2 image is missing")
    root = info["asset_root"]
    humble = NVIDIA / "humble_ws/src"
    dds = {"ROS_DOMAIN_ID": str(domain), "ROS_LOCALHOST_ONLY": "1",
           "RMW_IMPLEMENTATION": rmw}
    if rmw == "rmw_cyclonedds_cpp":
        dds["ROS_LOCALHOST_ONLY"] = "0"  # The explicit DDS profile binds only to loopback.
        dds["CYCLONEDDS_URI"] = (CHALLENGE / "config/cyclonedds.xml").as_uri()
    env = {**os.environ, **dds}
    for key in ("ROS_DISTRO", "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "PYTHONPATH",
                "OPENAI_API_KEY", "OPENAI_BASE_URL"):
        env.pop(key, None)
    sim = [str(ISAAC / "python.sh"), str(CHALLENGE / "isaac/run_baseline.py"),
           root + "/" + SCENE, root, str(gpu)]
    if calibration:
        sim.append("--calibration")
    elif rgbd:
        sim.append("--rgbd")
    if layout:
        layout = layout.resolve(strict=True)
        for fixture in ("nav/map.yaml", "nav/params.yaml", "sim/layout.usda", "sim/spawn.json"):
            if not (layout / fixture).is_file():
                raise RuntimeError(f"Incomplete graybox export: {fixture}")
        sim.extend(["--layout", str(layout / "sim")])
    if record_overview:
        sim.append("--record-overview")
    if camera_yaw:
        sim.extend(["--diagnostic-camera-yaw", str(camera_yaw)])
    if protected:
        sim.extend(["--trajectory", str(run / "physx-trajectory.jsonl")])
    container = f"luna-nav-{run_id.lower()}"
    nav = ["docker", "run", "--rm", "--name", container, "--network", "host", "--ipc", "host",
           "--user", f"{os.getuid()}:{os.getgid()}", "--env", "HOME=/tmp",
           "-v", f"{CHALLENGE}/config:/challenge:ro",
           "-v", f"{humble}/navigation/carter_navigation:/carter:ro"]
    launch = "source /opt/ros/humble/setup.bash && exec ros2 launch " \
             "/challenge/headless_nav.launch.xml carter:=/carter"
    if layout:
        nav.extend(["-v", f"{layout}/nav:/luna_nav:ro"])
        launch += " map:=/luna_nav/map.yaml params_file:=/luna_nav/params.yaml"
    if protected:
        nav.extend(["-v", f"{CHALLENGE}/navigation:/luna_code/navigation:ro",
                    "-v", f"{CHALLENGE}/ros2:/luna_code/ros2:ro",
                    "-v", f"{run}:/luna_run"])
        launch += (" protected:=true use_composition:=False"
                   " scan_min_height:=-0.12 scan_max_height:=0.35")
    for key, value in dds.items():
        if key == "CYCLONEDDS_URI":
            value = "file:///challenge/cyclonedds.xml"
        nav.extend(["--env", f"{key}={value}"])
    nav += ["--entrypoint", "/bin/bash", SETTINGS["nav2_image"], "-c", launch]
    probe = ["bash", "-c", 'source "$1" && shift && exec /usr/bin/python3 "$@"',
             "probe", str(ROS_SETUP), str(CHALLENGE / "scripts/readiness.py"),
             str(run), str(timeout)]
    if protected:
        probe.append("--protected")
    recorder = ["bash", "-c", 'source "$1" && shift && exec /usr/bin/python3 "$@"',
                "record", str(ROS_SETUP), str(CHALLENGE / "video/record.py"), str(run / "overview")]
    commands = [sim, nav, probe] + ([recorder] if record_overview else [])
    write_json(run / "manifest.json", {"run_id": run_id, "source": "isaac-live",
                                       "dds": dds, "commands": commands,
                                       "nav2_container": container, "gpu_index": gpu,
                                       "protected": protected, "diagnostic_camera_yaw_deg": camera_yaw,
                                       "layout": str(layout) if layout else None})
    try:
        with Processes(run) as processes:
            simulation = processes.spawn("isaac", sim, env)
            navigation = processes.spawn("nav2", nav, env)
            readiness = processes.spawn("readiness", probe, env)
            if record_overview:
                processes.spawn("overview", recorder, env)
            while True:
                for name, child in (("Isaac", simulation), ("Nav2", navigation)):
                    if child.poll() is not None:
                        raise RuntimeError(f"{name} exited with {child.returncode}. See {run}")
                if readiness.poll() is not None:
                    if readiness.returncode:
                        raise RuntimeError(f"Readiness failed with {readiness.returncode}. See {run}")
                    break
                time.sleep(0.25)
            print(f"READY {run}", flush=True)
            while simulation.poll() is None and navigation.poll() is None:
                time.sleep(0.5)
            raise RuntimeError(f"A live component exited. See {run}")
    finally:
        # Scope cleanup to this exact attempt even if the Docker CLI detached.
        if capture(["docker", "container", "inspect", "--format", "{{.Id}}",
                    container]).get("exit_code") == 0:
            capture(["docker", "rm", "-f", container])
        write_json(run / "container-stop.json", {
            "removed": capture(["docker", "container", "inspect", "--format",
                                "{{.Id}}", container]).get("exit_code") == 1,
        })


def stop(expected=None):
    active = RUNTIME / "active.json"
    if not active.exists():
        print("No active Luna navigation supervisor")
        return
    try:
        record = json.loads(active.read_text())
    except FileNotFoundError:
        print("No active Luna navigation supervisor")
        return
    if expected is not None and record != expected:
        return
    pid = record["pid"]
    try:
        with os.fdopen(os.pidfd_open(pid), "rb") as process_fd:
            if birth_ticks(pid) != record["birth_ticks"]:
                raise RuntimeError("Supervisor PID no longer belongs to the recorded attempt")
            signal.pidfd_send_signal(process_fd.fileno(), signal.SIGTERM)
    except ProcessLookupError:
        print("Recorded supervisor has already exited")
        return
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        try:
            current = json.loads(active.read_text())
        except FileNotFoundError:
            current = None
        if current != record:
            print(Path(record["run"]) / "stop.json")
            return
        time.sleep(0.2)
    raise RuntimeError("Supervisor did not finish cleanup within 45 seconds")


def observe(count, timeout, expected=None):
    try:
        record = json.loads((RUNTIME / "active.json").read_text())
    except FileNotFoundError:
        raise RuntimeError("Start a Luna simulation with --rgbd first") from None
    if expected is not None and record != expected:
        raise RuntimeError("Active episode changed before RGB-D capture")
    if birth_ticks(record["pid"]) != record["birth_ticks"]:
        raise RuntimeError("The recorded Luna supervisor has exited")
    run = Path(record["run"])
    manifest = json.loads((run / "manifest.json").read_text())
    calibration = "--calibration" in manifest["commands"][0]
    if "--rgbd" not in manifest["commands"][0] and not calibration:
        raise RuntimeError("The active simulation has no Luna RGB-D render product")
    directory = run / ("rgbd-" + uuid4().hex[:8])
    print(directory, flush=True)
    env = {**os.environ, **manifest["dds"]}
    for key in ("OPENAI_API_KEY", "OPENAI_BASE_URL"):
        env.pop(key, None)
    result = subprocess.run(
        ["bash", "-c", 'source "$1" && shift && exec /usr/bin/python3 "$@"',
         "observe", str(ROS_SETUP), str(CHALLENGE / "ros2/observation_node.py"),
         str(directory), manifest["run_id"], "--count", str(count), "--timeout", str(timeout)]
        + (["--calibration"] if calibration else [])
        + ["--diagnostic-camera-yaw", str(manifest.get("diagnostic_camera_yaw_deg", 0.0))],
        env=env,
    )
    if result.returncode:
        raise RuntimeError(f"RGB-D capture failed. See {directory}")
    return directory


def motion_episode():
    try:
        active = json.loads((RUNTIME / "active.json").read_text())
    except FileNotFoundError:
        raise RuntimeError("Start the fork in protected mode first") from None
    if birth_ticks(active["pid"]) != active["birth_ticks"]:
        raise RuntimeError("The recorded Luna supervisor has exited")
    run = Path(active["run"])
    manifest = json.loads((run / "manifest.json").read_text())
    if manifest.get("diagnostic_camera_yaw_deg"):
        raise RuntimeError("Diagnostic camera runs do not accept motion goals")
    if not manifest["protected"] or not json.loads((run / "readiness.json").read_text())["ready"]:
        raise RuntimeError("This experiment requires a ready protected fork")
    return active, run, manifest


def claim_motion(lock):
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError("A test already owns this episode") from None


def execute_motion(active, run, manifest, gate, cancel_after=None, live_observation=None):
    if birth_ticks(active["pid"]) != active["birth_ticks"]:
        raise RuntimeError("Active episode ended before motion")
    evidence = "controller-" + uuid4().hex[:8] + ".jsonl"
    command = ["docker", "exec", manifest["nav2_container"], "bash", "-c",
               'source /opt/ros/humble/setup.bash && exec python3 "$@"',
               "controller-test", "/luna_code/ros2/controller_test.py", gate,
               "/luna_run/" + evidence]
    if cancel_after is not None:
        command.extend(["--cancel-after", str(cancel_after)])
    if live_observation is not None:
        relative = live_observation.relative_to(run)
        command.extend(["--live-observation", "/luna_run/" + str(relative),
                        "--episode", manifest["run_id"]])
    print(run / evidence, flush=True)
    succeeded = False
    try:
        succeeded = subprocess.run(command, timeout=120).returncode == 0
    finally:
        if not succeeded:
            # A lost action response still leaves the simulator under this supervisor.
            stop(expected=active)
    if not succeeded:
        raise RuntimeError(f"Controller failed. See {run / evidence}")
    return run / evidence


def controller_test(gate, cancel_after):
    active, run, manifest = motion_episode()
    with (run / "controller-test.lock").open("a") as lock:
        claim_motion(lock)
        execute_motion(active, run, manifest, gate, cancel_after)


def luna_run(key, api_timeout):
    sys.path.insert(0, str(CHALLENGE))
    from evaluator.controller_run import evaluate
    from navigation.luna_probe import load_frame, probe

    active, run, manifest = motion_episode()
    layout = Path(manifest["layout"]) / "nav" / "public_scene_manifest.json"
    with (run / "controller-test.lock").open("a") as lock:
        claim_motion(lock)
        capture_dir = observe(1, 15, expected=active)
        observation, _ = load_frame(capture_dir, 1)
        model_dir = capture_dir / "luna-live"
        print(model_dir, flush=True)
        intent = probe(capture_dir, 1, layout, model_dir, SETTINGS["model"], api_timeout, key)
        print(json.dumps(intent, ensure_ascii=False), flush=True)
        if intent["decision"] != "select_candidate":
            print("Luna requested another observation; no motion goal submitted", flush=True)
            return
        observation_path = model_dir / "intent-observation.json"
        write_json(observation_path, observation)
        events = execute_motion(active, run, manifest, intent["candidate_id"],
                                live_observation=observation_path)
        deadline = time.monotonic() + 3
        while True:
            result = evaluate(run, events, layout)
            if result["pass"] or time.monotonic() >= deadline:
                break
            time.sleep(0.15)
        result["verified_host_mono_ns"] = str(time.monotonic_ns())
        write_json(model_dir / "physx-evaluation.json", result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if not result["pass"]:
            raise RuntimeError(f"PhysX route verification failed. See {model_dir}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    environment = commands.add_parser("doctor")
    environment.add_argument("--output", type=Path)
    launch = commands.add_parser("start")
    launch.add_argument("--domain", type=int, default=SETTINGS["ros_domain_id"])
    launch.add_argument("--gpu", type=int, default=SETTINGS["gpu_index"])
    launch.add_argument("--timeout", type=float, default=SETTINGS["readiness_timeout_s"])
    launch.add_argument("--rgbd", action="store_true")
    launch.add_argument("--record-overview", action="store_true")
    launch.add_argument("--layout", type=Path)
    launch.add_argument("--protected", action="store_true")
    launch.add_argument("--rmw", choices=["rmw_fastrtps_cpp", "rmw_cyclonedds_cpp"])
    launch.add_argument("--calibration", action="store_true")
    launch.add_argument("--diagnostic-camera-yaw", type=float, default=0.0)
    sensor = commands.add_parser("observe")
    sensor.add_argument("--count", type=int, default=5)
    sensor.add_argument("--timeout", type=float, default=45)
    controller = commands.add_parser("controller-test")
    controller.add_argument("gate")
    controller.add_argument("--cancel-after", type=float)
    visual = commands.add_parser("luna-probe")
    visual.add_argument("capture", type=Path)
    visual.add_argument("layout", type=Path)
    visual.add_argument("output", type=Path)
    visual.add_argument("--frame", type=int, default=1)
    visual.add_argument("--model", default=SETTINGS["model"])
    visual.add_argument("--reasoning", choices=["none", "low"], default="none")
    visual.add_argument("--timeout", type=float, default=25)
    live = commands.add_parser("luna-run")
    live.add_argument("--api-timeout", type=float, default=5)
    commands.add_parser("stop")
    args = parser.parse_args()
    try:
        if args.command == "doctor":
            report = doctor()
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                write_json(args.output, report)
            else:
                print(json.dumps(report, ensure_ascii=False, indent=2))
        elif args.command == "start":
            if not 0 <= args.domain <= 232 or args.gpu < 0 or not 0 < args.timeout <= 900:
                parser.error("Expected domain 0–232, nonnegative GPU and timeout in (0, 900]")
            if args.protected and not args.layout:
                parser.error("--protected requires a SimScene layout")
            if not -180 <= args.diagnostic_camera_yaw <= 180:
                parser.error("Expected camera yaw in [-180, 180]")
            if args.diagnostic_camera_yaw and (not args.rgbd or not args.layout or args.calibration):
                parser.error("Diagnostic camera yaw requires a graybox RGB-D run")
            if args.record_overview and not args.layout:
                parser.error("--record-overview requires a SimScene layout")
            start(args.domain, args.gpu, args.timeout, args.rgbd, args.calibration,
                  args.layout, args.protected,
                  args.rmw or ("rmw_cyclonedds_cpp" if args.protected else "rmw_fastrtps_cpp"),
                  args.diagnostic_camera_yaw, args.record_overview)
        elif args.command == "observe":
            if args.count < 1 or not 0 < args.timeout <= 900:
                parser.error("Expected positive sample count and timeout in (0, 900]")
            observe(args.count, args.timeout)
        elif args.command == "controller-test":
            if args.cancel_after is not None and not 0 < args.cancel_after < 75:
                parser.error("--cancel-after must be between 0 and 75 seconds")
            controller_test(args.gate, args.cancel_after)
        elif args.command == "luna-run":
            key = os.environ.get("OPENAI_API_KEY")
            if not key:
                raise RuntimeError("OPENAI_API_KEY is required for live Luna")
            if not 0.05 <= args.api_timeout <= 5:
                parser.error("Expected API timeout in [0.05, 5] seconds")
            luna_run(key, args.api_timeout)
        elif args.command == "luna-probe":
            if args.frame < 1 or not 0 < args.timeout <= 120:
                parser.error("Expected a positive frame and timeout in (0, 120]")
            key = os.environ.get("OPENAI_API_KEY")
            if not key:
                raise RuntimeError("OPENAI_API_KEY is required for the Luna probe")
            sys.path.insert(0, str(CHALLENGE))
            from navigation.luna_probe import probe

            intent = probe(args.capture, args.frame, args.layout, args.output,
                           args.model, args.timeout, key, args.reasoning)
            print(json.dumps(intent, ensure_ascii=False))
        else:
            stop()
    except (OSError, RuntimeError, ValueError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
