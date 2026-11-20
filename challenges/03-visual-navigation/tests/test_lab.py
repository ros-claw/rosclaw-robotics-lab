import importlib.util
from pathlib import Path
import sys

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts/lab.py"
spec = importlib.util.spec_from_file_location("visual_lab", SCRIPT)
lab = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = lab
spec.loader.exec_module(lab)


def test_asset_root_is_from_the_requested_isaac_version(tmp_path):
    config = tmp_path / "exts/isaacsim.storage.native/config/extension.toml"
    config.parent.mkdir(parents=True)
    config.write_text(
        'exts."isaacsim.storage.native".asset_region_profiles.us.asset_root '
        '= "https://example.org/Assets/Isaac/6.2"\n'
    )
    assert lab.asset_root(tmp_path, "us", None) == "https://example.org/Assets/Isaac/6.2"
    with pytest.raises(ValueError, match="6.2"):
        lab.asset_root(tmp_path, "us", "https://example.org/Assets/Isaac/6.1")


def test_failure_cleans_up_only_owned_processes(tmp_path):
    import subprocess

    other = subprocess.Popen(["sleep", "30"], start_new_session=True)
    try:
        with pytest.raises(RuntimeError, match="readiness failed"):
            with lab.Processes(tmp_path) as run:
                owned = run.spawn("test", ["sleep", "30"])
                raise RuntimeError("readiness failed")
        assert owned.poll() is not None
        assert other.poll() is None
    finally:
        other.terminate()
        other.wait(timeout=5)
