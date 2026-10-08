"""Synthetic negative checks exercise the gate; they are not simulation evidence."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'evaluator'))
from obstacle import verify_obstacle


class ObstacleGateTests(unittest.TestCase):
    def fixture(self, root):
        (root/'scene-audit.json').write_text(json.dumps({'unmapped_test_obstacle':{'size_m':1,'center_xyz':[-3.5,4,.5],'physical_collision':True}}))
        (root/'path-witness.jsonl').write_text(json.dumps({'wall_time':1,'box_hit_count':30,'occupied_cells':20})+'\n')
        return [{'physics_transforms_xyzw':[[-6,-1,0,0,0,0,1]],'collision_count':0,'collision_observer_complete':True}]

    def test_complete_contract(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);samples=self.fixture(root)
            self.assertEqual(verify_obstacle(root,samples,0,2,{'all_box_cells_free':True})['status'],'PASS')

    def test_lidar_alone_cannot_prove_avoidance(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);samples=self.fixture(root)
            (root/'path-witness.jsonl').write_text(json.dumps({'wall_time':1,'box_hit_count':30,'occupied_cells':0})+'\n')
            self.assertEqual(verify_obstacle(root,samples,0,2,{'all_box_cells_free':True})['status'],'FAIL')

    def test_collision_cannot_pass(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);samples=self.fixture(root);samples[0]['collision_count']=1
            self.assertEqual(verify_obstacle(root,samples,0,2,{'all_box_cells_free':True})['status'],'FAIL')

    def test_footprint_overlap_cannot_pass(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);samples=self.fixture(root);samples[0]['physics_transforms_xyzw'][0][:2]=[-3.5,4]
            self.assertEqual(verify_obstacle(root,samples,0,2,{'all_box_cells_free':True})['status'],'FAIL')


if __name__=='__main__':
    unittest.main()
