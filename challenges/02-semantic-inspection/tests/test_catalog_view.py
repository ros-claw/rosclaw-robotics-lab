import copy
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from catalog_view import compact_catalog


class CatalogBudgetTests(unittest.TestCase):
    def test_full_eight_target_catalog_keeps_return_requirements_under_native_budget(
        self,
    ):
        catalog = {
            "return_to_initial_pose": {
                "proposal_id": "f" * 64,
                "x": -6.012007713317871,
                "y": -1.0,
                "yaw": 3.141562737927037,
                "target_prim": "return_to_initial_pose",
            },
            "initial_pose": {"x": -6.0, "y": -1.0, "yaw": 3.14},
            "generated_at_wall": 1791576185.0,
            "proposal_max_wall_age_seconds": 90,
            "body_snapshot_hash": "b" * 64,
            "inspection_scope": "semantic observation-point navigation only",
            "targets": [],
        }
        for i in range(8):
            catalog["targets"].append(
                {
                    "target_prim": "/World/warehouse_with_forklifts/Warehouse_Empty_small_realtime/SM_RackShelf_"
                    + str(158 + i),
                    "center_xy": [-9.320000306665882, -1.7000048448147718],
                    "distance_from_initial_pose_m": 3.3812455959911363,
                    "west_of_initial_pose": True,
                    "status": "PROPOSAL_ONLY",
                    "proposals": [
                        {
                            "x": -6.94514422865085,
                            "y": -0.7000045353522055,
                            "yaw": -2.743048423166946,
                            "proposal_id": str(n) * 64,
                            "target_prim": "/World/long_source_name",
                            "static_clearance_m": 1.4646446609406727,
                        }
                        for n in range(8)
                    ],
                }
            )
        physics = {
            "observer_id": "o" * 32,
            "wall_time": 1791576185.0,
            "sim_time": 12.4,
            "physics_transforms_xyzw": [[-6, -1, 0, 0, 0, 1, 0]],
            "timeline_playing": True,
            "collision_count": 0,
        }
        original = copy.deepcopy(catalog)
        view = compact_catalog(catalog, physics)
        wire = json.dumps(
            {
                "status": "SUCCEEDED",
                "capability_id": "semantic.observe_candidates",
                "value": view,
            },
            ensure_ascii=False,
        )
        self.assertLessEqual(len(wire), 8000)
        decoded = json.loads(wire)["value"]
        self.assertEqual(decoded["return_to_initial_pose"]["proposal_id"], "f" * 64)
        self.assertEqual(len(decoded["targets"]), 8)
        self.assertTrue(decoded["requirements"])
        self.assertEqual(
            decoded["targets"][0]["proposals"][0]["proposal_id"],
            catalog["targets"][0]["proposals"][0]["proposal_id"],
        )
        self.assertEqual(catalog, original)


if __name__ == "__main__":
    unittest.main()
