import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from safety import swept_footprint, lidar_target_hits


class SweptFootprintTests(unittest.TestCase):
    def setUp(self):
        self.grid = {
            "resolution": 0.1,
            "width": 100,
            "height": 100,
            "origin": [-5, -5],
            "data": [0] * 10000,
        }
        self.footprint = [[-0.6, -0.25], [0.14, -0.25], [0.14, 0.25], [-0.6, 0.25]]

    def test_free_with_interpolated_rotation(self):
        proof = swept_footprint([[0, 0, 0], [2, 0, 1.57]], self.grid, self.footprint)
        self.assertGreater(proof["sampled_poses"], 40)

    def test_obstacle_between_sparse_waypoints(self):
        self.grid["data"][50 * 100 + 60] = 100
        with self.assertRaisesRegex(ValueError, "intersects"):
            swept_footprint([[0, 0, 0], [2, 0, 0]], self.grid, self.footprint)

    def test_footprint_corner_not_center(self):
        self.grid["data"][52 * 100 + 47] = 100
        with self.assertRaisesRegex(ValueError, "intersects"):
            swept_footprint([[0, 0, 0]], self.grid, self.footprint)

    def test_unknown_and_inscribed_rejected(self):
        for cost in [-1, 99]:
            self.grid["data"][50 * 100 + 50] = cost
            with self.assertRaises(ValueError):
                swept_footprint([[0, 0, 0]], self.grid, self.footprint)

    def test_nonlethal_inflation_not_double_counted(self):
        self.grid["data"] = [80] * 10000
        self.assertEqual(
            swept_footprint([[0, 0, 0]], self.grid, self.footprint)[
                "maximum_nonlethal_cost"
            ],
            80,
        )

    def test_outside_and_nan_rejected(self):
        for path in [[[4.99, 0, 0]], [[0, 0, float("nan")]]]:
            with self.assertRaises(ValueError):
                swept_footprint(path, self.grid, self.footprint)

    def test_rotation_sweeps_obstacle(self):
        # Long rear footprint rotates into this cell; both endpoint polygons miss.
        self.grid["data"][46 * 100 + 46] = 100
        with self.assertRaises(ValueError):
            swept_footprint(
                [[0, 0, 0], [0, 0, 1.57]], self.grid, self.footprint, padding=0
            )


class LidarInspectionTests(unittest.TestCase):
    def setUp(self):
        self.scan = {
            "stamp": 10,
            "tf_stamp": 10,
            "map_sensor_xy_yaw": [0, 0, 0],
            "angle_min": -0.01,
            "angle_increment": 0.01,
            "range_min": 0.1,
            "range_max": 20,
            "ranges": [2, 2, 2, None],
        }
        self.bounds = [[1.9, -0.2], [2.1, 0.2]]

    def test_measured_returns_in_known_region(self):
        self.assertEqual(
            lidar_target_hits(self.scan, self.bounds, sim_time=10)["hit_count"], 3
        )

    def test_stale_or_unaligned_evidence_rejected(self):
        for key in ["stamp", "tf_stamp"]:
            scan = copy.deepcopy(self.scan)
            scan[key] = 1
            with self.assertRaises(ValueError):
                lidar_target_hits(scan, self.bounds, sim_time=10)

    def test_no_target_returns_rejected(self):
        self.scan["ranges"] = [1, 1, 1, None]
        with self.assertRaisesRegex(ValueError, "Insufficient"):
            lidar_target_hits(self.scan, self.bounds, sim_time=10)


if __name__ == "__main__":
    unittest.main()
