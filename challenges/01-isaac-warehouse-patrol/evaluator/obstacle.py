"""Independent unmapped-box evidence gate; neither Agent prose nor arrival alone."""
import json
import math
from pathlib import Path


def verify_obstacle(physics, trajectories, start, end, map_validation=None):
    physics=Path(physics)
    audit=json.loads((physics/'scene-audit.json').read_text())
    box=audit.get('unmapped_test_obstacle')
    failures=[]
    if not map_validation or not map_validation.get('all_box_cells_free'):
        failures.append('original static-map free-cell proof missing')
    if not box or box.get('size_m')!=1 or box.get('center_xyz')!=[-3.5,4,.5] or not box.get('physical_collision'):
        failures.append('actual expected session-layer obstacle missing')
    witness=physics/'path-witness.jsonl'
    rows=[json.loads(line) for line in witness.read_text().splitlines()] if witness.exists() else []
    rows=[r for r in rows if start<=r['wall_time']<=end]
    hits=max((r.get('box_hit_count',0) for r in rows),default=0)
    occupied=max((r.get('occupied_cells',0) for r in rows),default=0)
    if hits<=0:
        failures.append('actual LiDAR box observations missing')
    if occupied<=0:
        failures.append('actual local master costmap did not mark unmapped box')
    # Conservative circle enclosing the registered Nav2 physical footprint.
    radius=math.hypot(.607,.25)
    distances=[]
    for sample in trajectories:
        x,y=sample['physics_transforms_xyzw'][0][:2]
        dx=max(abs(x+3.5)-.5,0);dy=max(abs(y-4)-.5,0)
        distances.append(math.hypot(dx,dy)-radius)
        if sample.get('collision_count') or not sample.get('collision_observer_complete'):
            failures.append('collision or incomplete contact evidence')
            break
    clearance=min(distances,default=-math.inf)
    if clearance<0:
        failures.append('conservative robot footprint overlaps obstacle')
    return {'status':'FAIL' if failures else 'PASS','failures':failures,
            'maximum_actual_lidar_box_hits':hits,'maximum_actual_local_occupied_cells':occupied,
            'conservative_minimum_footprint_clearance_m':clearance if math.isfinite(clearance) else None,
            'wall_interval':[start,end],'witness_subset_sha256':__import__('hashlib').sha256(('\n'.join(json.dumps(r,sort_keys=True,separators=(',',':')) for r in rows)).encode()).hexdigest(),
            'witness_subset_row_count':len(rows),
            'physical_obstacle':box,'static_map_validation':map_validation}
