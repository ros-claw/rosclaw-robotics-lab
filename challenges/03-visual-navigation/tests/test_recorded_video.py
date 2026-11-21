from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def test_manifest_uses_recorded_wall_intervals():
    from video.export import concat_manifest

    rows = [{"name": "0001.jpg", "host_mono_ns": 3_000_000_000},
            {"name": "0002.jpg", "host_mono_ns": 3_170_000_000},
            {"name": "0003.jpg", "host_mono_ns": 3_270_000_000}]
    manifest = concat_manifest(rows)
    assert "duration 0.170000000" in manifest
    assert "duration 0.100000000" in manifest
    assert manifest.count("file ") == 3
    with pytest.raises(ValueError):
        concat_manifest(rows[::-1])
