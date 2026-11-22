from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation.pooled_https import PooledHTTPS


def test_pooled_transport_releases_the_session_and_rejects_use_after_close():
    with PooledHTTPS() as connection:
        assert connection.handle
    assert not connection.handle
    with pytest.raises(RuntimeError, match="closed"):
        connection({}, "placeholder", 1)
