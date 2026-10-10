"""Isolated execution owner. Native Agent never constructs this Runtime."""

import argparse
import json
import logging
import os
import signal
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "evaluator"))
from point_executor import PointExecutor, ScanWitness
from evidence_io import write_json_atomic
from remember import PatrolMemoryExecutor
from rosclaw.connectors.ros.action_client import Ros2ActionClient
from rosclaw.connectors.ros.practice import RosPracticeAdapter
from rosclaw.connectors.ros.transport.base import RosbridgeEndpoint
from rosclaw.connectors.ros.transport.rosbridge import RosbridgeTransport
from rosclaw.core.runtime import Runtime, RuntimeConfig
from rosclaw.daemon.ledger import DaemonLedger
from rosclaw.daemon.server import RosclawDaemon
from rosclaw.daemon.service import DaemonControlPlane
from rosclaw.kernel import ExecutionMode
from rosclaw.practice.recorder import PracticeRecorder
from rosclaw.runtime.bus import RuntimeBus
from rosclaw.runtime.event import RuntimeEvent


def main():
    logging.basicConfig(level=logging.INFO)
    p = argparse.ArgumentParser()
    p.add_argument("--directory", type=Path, required=True)
    p.add_argument("--endpoint", default="ws://127.0.0.1:19091")
    a = p.parse_args()
    root = a.directory.resolve()
    config = json.loads((root / "execution_config.json").read_text())
    runtime = Runtime(
        RuntimeConfig(
            robot_id=config["body_id"],
            workspace_home=str(root / "home"),
            enable_firewall=False,
            enable_memory=True,
            enable_practice=False,
            enable_skill_manager=False,
            enable_knowledge=False,
            enable_how=False,
            enable_auto=False,
            enable_provider=False,
            enable_sense=False,
            enable_recovery_loop=False,
            enable_event_persistence=False,
            enable_tracing=False,
            seekdb_backend="sqlite",
            seekdb_path=str(root / "memory.sqlite"),
        )
    )
    runtime.initialize()
    runtime.start()
    practice = RosPracticeAdapter(runtime.event_bus)
    practice.initialize()
    bus = RuntimeBus(event_bus=runtime.event_bus)
    bus.robot_id = config["body_id"]
    recorder = PracticeRecorder(
        bus, data_root=root / "practice", publish_to_event_bus=False
    )
    recorder.initialize()
    recorder.start()
    bus.publish(
        RuntimeEvent(
            type="practice.start",
            source="runtime",
            robot=config["body_id"],
            body_id=config["body_id"],
            payload={
                "practice_id": "isaac-warehouse-patrol",
                "episode_id": root.name,
                "metadata": {
                    "evidence_domain": "SIMULATION",
                    "engine": "isaacsim_physx",
                },
            },
        )
    )
    transports = [
        RosbridgeTransport(RosbridgeEndpoint.from_url(a.endpoint)) for _ in range(2)
    ]
    for transport in transports:
        response = transport.connect()
        if not response.ok:
            raise ConnectionError(response.error)
    client = Ros2ActionClient(transports[0])
    sensor = ScanWitness(transports[1])
    executor_type = PointExecutor
    memory_type = PatrolMemoryExecutor
    if config.get("scenario") == "semantic_observation":
        sys.path.insert(0, config["semantic_source_directory"])
        from semantic_executor import SemanticExecutor, SemanticMemoryExecutor
        executor_type, memory_type = SemanticExecutor, SemanticMemoryExecutor
    if config.get("scenario") == "loading_inspection":
        sys.path.insert(0, config["semantic_source_directory"])
        from loading_executor import LoadingExecutor, LoadingMemoryExecutor
        executor_type, memory_type = LoadingExecutor, LoadingMemoryExecutor
    executor = executor_type(root=root, config=config, client=client, sensor=sensor)
    executor.fresh()
    runtime.action_gateway.register_executor(
        "navigation.navigate_to_pose", ExecutionMode.SIMULATION, executor
    )
    runtime.register_driver("isaac_nav2", executor)
    runtime.action_gateway.register_executor(
        config.get("verification_capability", "patrol.verify_and_remember"),
        ExecutionMode.SIMULATION,
        memory_type(runtime, root, config, bus),
    )
    ledger = DaemonLedger(root / "state/control.sqlite")
    daemon = RosclawDaemon(
        service=DaemonControlPlane(
            runtime=runtime, state_dir=root / "state", ledger=ledger
        ),
        socket_path=root / "run/rosclawd.sock",
    )
    stopped = threading.Event()
    for sig in [signal.SIGINT, signal.SIGTERM]:
        signal.signal(sig, lambda *_: stopped.set())
    try:
        daemon.start()
        write_json_atomic(root / "daemon_ready.json", {"pid": os.getpid(), "body_id": config["body_id"]})
        stopped.wait()
    finally:
        executor.emergency_stop()
        if hasattr(executor, "close"):
            executor.close()
        daemon.stop()
        sensor.close()
        client.close()
        if recorder.session is not None:
            bus.publish(
                RuntimeEvent(
                    type="practice.stop",
                    source="runtime",
                    robot=config["body_id"],
                    body_id=config["body_id"],
                    payload={
                        "outcome": "FAILURE",
                        "failure_labels": ["mission_not_verified"],
                    },
                )
            )
        practice.stop()
        recorder.stop()
        runtime.stop()
        ledger.close()


if __name__ == "__main__":
    main()
