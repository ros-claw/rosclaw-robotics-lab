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


class IndependentInspectionTests(unittest.TestCase):
    def test_sensor_returns_require_stopped_facing_collision_free_independent_body(
        self,
    ):
        from safety import inspect_known_region

        scan = {
            "stamp": 10,
            "tf_stamp": 10,
            "map_sensor_xy_yaw": [0, 0, 0],
            "frame": "front_3d_lidar",
            "angle_min": -0.01,
            "angle_increment": 0.01,
            "range_min": 0.1,
            "range_max": 20,
            "ranges": [2, 2, 2, None],
        }
        snapshot = {"wall_time": 100, "sim_time": 10, "scan": scan}
        physics = {
            "wall_time": 100,
            "sim_time": 10,
            "physics_body_path": "body",
            "observer_id": "reset",
            "timeline_playing": True,
            "collision_observer_complete": True,
            "collision_count": 0,
            "contact_errors": [],
            "physics_transforms_xyzw": [[0, 0, 0, 0, 0, 0, 1]],
            "linear_velocity_xyz": [0, 0, 0],
            "angular_velocity_xyz": [0, 0, 0],
        }
        site = {"x": 0, "y": 0, "yaw": 0}
        bounds = [[1.9, -0.2], [2.1, 0.2]]
        self.assertEqual(
            inspect_known_region(
                snapshot, physics, site, bounds, body_path="body", observer_id="reset"
            )["hit_count"],
            3,
        )
        for patch in [
            {"collision_count": 1},
            {"observer_id": "another-reset"},
            {"timeline_playing": False},
            {"wall_time": 90},
            {"linear_velocity_xyz": [0.3, 0, 0]},
            {"physics_transforms_xyzw": [[0, 0, 0, 0, 0, 1, 0]]},
        ]:
            with self.assertRaises(ValueError):
                inspect_known_region(
                    snapshot,
                    {**physics, **patch},
                    site,
                    bounds,
                    body_path="body",
                    observer_id="reset",
                )
