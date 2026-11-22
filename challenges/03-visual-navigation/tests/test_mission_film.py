from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from video.caption import concise, picture_windows


def test_film_does_not_show_a_frame_before_its_request():
    samples = [(1_500_000_000, "one.png"), (2_000_000_000, "two.png")]
    assert picture_windows(samples, 1_000_000_000) == [
        (0.5, 1.0, "one.png"), (1.0, 5.0, "two.png"),
    ]
    assert concise("前方遮挡\n需要复核") == "前方遮挡 需要复核"
