"""Freeze only public source/config, never credentials or runtime signing keys."""

import hashlib
import json
import os
import subprocess
import shutil
from pathlib import Path


def freeze(root, challenge):
    files = [
        p
        for folder in ["isaac", "ros2", "rosclaw", "evaluator", "config", "scripts"]
        for p in (challenge / folder).rglob("*")
        if p.is_file() and "__pycache__" not in p.parts and p.suffix not in {".pyc"}
    ]
    manifest = {
        "git_sha": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=challenge, text=True
        ).strip(),
        "working_tree_dirty": bool(
            subprocess.check_output(["git", "status", "--porcelain"], cwd=challenge)
        ),
        "source_hashes": {
            str(p.relative_to(challenge)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in files
        },
        "execution_config_sha256": hashlib.sha256(
            (root / "execution_config.json").read_bytes()
        ).hexdigest(),
    }
    config = json.loads((root / "execution_config.json").read_text())
    if config.get("scenario") in {"semantic_observation", "loading_inspection"}:
        semantic = Path(config["semantic_source_directory"])
        semantic_files = [p for p in semantic.glob("*.py") if p.is_file()]
        manifest["semantic_source_hashes"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in semantic_files}
        for p in semantic_files:
            target = root / "frozen-semantic-source" / p.name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(p, target)
    upstream = Path(
        os.environ.get("ROSCLAW_SOURCE", str(Path.home() / "sim/rosclaw-upstream"))
    )
    manifest["rosclaw_upstream"] = {
        "git_sha": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=upstream, text=True
        ).strip(),
        "working_tree_dirty": bool(
            subprocess.check_output(["git", "status", "--porcelain"], cwd=upstream)
        ),
    }
    stamp = upstream / "packages/rosclaw-agent/dist/build-stamp.json"
    if stamp.exists():
        manifest["native_build_stamp"] = json.loads(stamp.read_text())
        manifest["native_build_sha256"] = {str(p.relative_to(upstream)): hashlib.sha256(p.read_bytes()).hexdigest() for p in [stamp, upstream / "packages/rosclaw-agent/dist/src/main.js", upstream / "packages/rosclaw-agent/dist/src/extension/index.js"]}
    try:
        manifest["docker_image"] = json.loads(
            subprocess.check_output(
                [
                    "docker",
                    "image",
                    "inspect",
                    os.environ.get(
                        "ROS_CONTAINER_IMAGE", "rosclaw/warehouse-jazzy:6.1.0"
                    ),
                ],
                text=True,
            )
        )[0]["Id"]
    except subprocess.CalledProcessError:
        manifest["docker_image"] = "UNKNOWN"
    for path in files:
        target = root / "frozen-source" / path.relative_to(challenge)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(path, target)
    (root / "source-freeze.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest
