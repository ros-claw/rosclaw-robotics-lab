"""Map recorded public evidence paths into a replay root without rewriting bytes."""

import hashlib
from pathlib import Path


def artifact_path(root, config, artifact, *, directory):
    recorded = Path(artifact["path"])
    original = Path(config["execution_root"]) / directory
    try:
        relative = recorded.relative_to(original)
    except ValueError as exc:
        raise ValueError("Evidence path outside recorded scope") from exc
    if len(relative.parts) != 1 or relative.name in {".", ".."}:
        raise ValueError("Evidence must be a direct file in its allowed directory")
    path = Path(root) / directory / relative.name
    if (
        path.is_symlink()
        or not path.is_file()
        or hashlib.sha256(path.read_bytes()).hexdigest() != artifact["sha256"]
    ):
        raise ValueError("Evidence artifact file/hash mismatch")
    return path
