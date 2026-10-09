import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

from PIL import Image, ImageDraw
import yaml

module = Path(__file__).resolve().parents[1] / 'propose_observation.py'
spec = importlib.util.spec_from_file_location('proposals', module)
planner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(planner)


class ProposalGateTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        image = Image.new('L', (160, 160), 254)
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, 159, 159), outline=0, width=4)
        draw.rectangle((70, 60, 89, 99), fill=0)
        image.save(self.root / 'map.png')
        (self.root / 'map.yaml').write_text(yaml.safe_dump({'image': 'map.png', 'resolution': .1, 'origin': [-8., -8., 0.], 'negate': 0, 'free_thresh': .196, 'occupied_thresh': .65}))
        self.target = next('/World/SM_RackShelf_' + str(i) for i in range(50)
            if int(__import__('hashlib').sha256(('/World/SM_RackShelf_' + str(i)).encode()).hexdigest()[:8], 16) % 4)
        self.write('inventory.json', {'up_axis': 'Z', 'meters_per_unit': 1, 'scene_objects': [{'path': self.target, 'type': 'Xform', 'min': [-1, -2, 0], 'max': [1, 2, 2]}]})
        self.write('physics.json', {'wall_time': 100., 'timeline_playing': True, 'collision_observer_complete': True, 'collision_count': 0, 'contact_errors': [], 'physics_body_path': '/World/Nova_Carter_ROS/chassis_link', 'physics_transforms_xyzw': [[-4., 0., 0., 0., 0., 0., 1.]], 'observer_id': 'fixture'})
        self.write('body.json', {'body_id': 'measured-sim-body', 'effective_body_hash': 'fixture-binding'})
        (self.root / 'params.yaml').write_text(yaml.safe_dump({'local_costmap': {'local_costmap': {'ros__parameters': {'footprint': '[[-0.6,-0.25],[0.14,-0.25],[0.14,0.25],[-0.6,0.25]]', 'footprint_padding': .03}}}}))

    def write(self, name, data):
        (self.root / name).write_text(json.dumps(data))

    def update(self, name, **changes):
        data = json.loads((self.root / name).read_text())
        data.update(changes)
        self.write(name, data)

    def plan(self, **kwargs):
        return planner.propose(inventory_path=self.root/'inventory.json', map_yaml=self.root/'map.yaml', physics_path=self.root/'physics.json', body_path=self.root/'body.json', nav_params=self.root/'params.yaml', target=self.target, now=100.5, **kwargs)

    def test_actual_bounds_generate_safe_hash_bound_candidates(self):
        result = self.plan()
        self.assertEqual(result['status'], 'PROPOSAL_ONLY')
        self.assertFalse(result['authorization'])
        self.assertFalse(result['execution_allowed'])
        for candidate in result['candidates']:
            self.assertGreaterEqual(candidate['static_clearance_m'], result['body_enclosing_radius_with_margins_m'])
            self.assertEqual(len(candidate['proposal_id']), 64)
        old = result['candidates'][0]['proposal_id']
        self.update('body.json', effective_body_hash='other-body')
        self.assertNotEqual(old, self.plan()['candidates'][0]['proposal_id'])

    def test_target_bounds_change_generated_positions(self):
        old = [(c['x'], c['y']) for c in self.plan()['candidates']]
        self.update('inventory.json', up_axis='Z', meters_per_unit=1, scene_objects=[{'path': self.target, 'type': 'Xform', 'min': [-1, -1, 0], 'max': [1, 1, 2]}])
        self.assertNotEqual(old, [(c['x'], c['y']) for c in self.plan()['candidates']])

    def test_unknown_cells_cannot_be_observation_goals(self):
        Image.new('L', (160, 160), 205).save(self.root / 'map.png')
        with self.assertRaisesRegex(ValueError, 'safe map component'):
            self.plan()

    def test_holdout_split_is_enforced(self):
        with self.assertRaisesRegex(ValueError, 'sealed alternate split'):
            self.plan(split='holdout')

    def test_stale_and_paused_physics_rejected(self):
        for changes in ({'wall_time': 90.}, {'timeline_playing': False}):
            self.update('physics.json', **changes)
            with self.assertRaisesRegex(ValueError, 'Fresh, playing'):
                self.plan()

    def test_collision_or_missing_observer_rejected(self):
        for changes in ({'collision_count': 1}, {'collision_observer_complete': False}):
            self.update('physics.json', **changes)
            with self.assertRaisesRegex(ValueError, 'collision observation'):
                self.plan()

    def test_missing_body_binding_rejected(self):
        self.update('body.json', effective_body_hash='')
        with self.assertRaisesRegex(ValueError, 'Body snapshot binding'):
            self.plan()

    def test_parent_and_child_shelf_cannot_leak_across_splits(self):
        inventory = {"scene_objects": [
            {"path": "/World/SM_RackShelf_1", "type": "Xform"},
            {"path": "/World/SM_RackShelf_1/SM_RackShelf_01", "type": "Xform"},
        ]}
        self.assertEqual(list(planner.target_split(inventory)), ["/World/SM_RackShelf_1"])

    def test_negative_margin_rejected(self):
        with self.assertRaisesRegex(ValueError, 'margins'):
            self.plan(margin=-.1)


if __name__ == '__main__':
    unittest.main()
