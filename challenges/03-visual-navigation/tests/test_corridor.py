from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation.candidates import Gate, ValidationError, validate_corridor


def test_committed_gate_cannot_be_bypassed():
    chosen = Gate("g-a1", 0.0, 1.2, 3.8, -0.5, 2.5)
    alternate = Gate("g-b2", 0.0, -3.8, -1.2, -0.5, 2.5)
    first = [(-4, 0), (-1.5, 2.5), (3, 2.5), (5, 0)]
    second = [(-4, 0), (-1.5, -2.5), (3, -2.5), (5, 0)]
    validate_corridor(first, chosen, (alternate,), 0.4, lambda *_: True)
    with pytest.raises(ValidationError):
        validate_corridor(second, chosen, (alternate,), 0.4, lambda *_: True)


def test_revalidates_entire_path_and_inflates_footprint():
    gate = Gate("g-a1", 0.0, 1.2, 3.8, -0.5, 2.5)
    with pytest.raises(ValidationError, match="clearance"):
        validate_corridor([(-4, 2.5), (4, 2.5)], gate, (), 0.4,
                          lambda x, y, radius: abs(x - 1.0) > radius)


def test_cannot_sweep_footprint_through_constraint_edge():
    gate = Gate("g-a1", 0.0, 1.2, 3.8, -0.5, 2.5)
    with pytest.raises(ValidationError, match="corridor"):
        validate_corridor([(-1, 1.4), (4, 1.4)], gate, (), 0.4, lambda *_: True)
