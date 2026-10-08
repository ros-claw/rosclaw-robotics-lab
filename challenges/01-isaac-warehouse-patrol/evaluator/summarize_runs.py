"""Aggregate explicit reported successes and failures without hiding failed trials."""
import argparse
import json
from pathlib import Path

p=argparse.ArgumentParser()
p.add_argument('--reports',type=Path,required=True)
p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
paths=sorted(a.reports.glob('native-acceptance-[1234].json'))
runs=[json.loads(path.read_text()) for path in paths]
success=[r for r in runs if r['status']=='PASS']
failures=[]
for path in sorted(a.reports.glob('*.json')):
    if path in paths:
        continue
    data=json.loads(path.read_text())
    if data.get('status') in {'FAIL','FAIL_PREFLIGHT','BLOCKED'}:
        failures.append({'report':path.name,'status':data['status'],'failure':data.get('failure',data.get('failures'))})
summary={'schema_version':'rosclaw.isaac_acceptance_summary.v1','successful_full_resets':len(success),
         'order_variation_passed':len({tuple(v['site_id'] for v in r['visits']) for r in success})>1,
         'all_four_sites_each_reset':all({v['site_id'] for v in r['visits']}=={'entry','shelf','aisle','home'} for r in success),
         'all_zero_non_floor_contacts':all(v['collision_count']==0 for r in success for v in r['visits']),
         'maximum_position_error_m':max((v['position_error_m'] for r in success for v in r['visits']),default=None),
         'model_provider_ids':sorted({model for r in success for model in r['models']}),
         'runs':[{'report':path.name,'run_id':r['run_id'],'status':r['status'],'order':[v['site_id'] for v in r['visits']],
                  'model_turns':r['model_turns'],'distance_m':r['actual_distance_m'],'physical_action_wall_seconds':r['physical_action_wall_seconds'],
                  'input_to_first_motion_wall_seconds':r['input_to_first_motion_wall_seconds'],
                  'observed_sim_to_wall_ratio':r.get('observed_sim_to_wall_ratio'),'body_hash':r['body_snapshot_hash'],
                  'source_manifest':r['source_freeze']} for path,r in zip(paths,runs)],
         'retained_failed_reports':failures,'statistical_success_rate':None,
         'independent_clean_machine_reproduction':'PENDING_EXTERNAL_ENGINEER',
         'hardware_verified':False,'notes':['Passing resets used documented source revisions during integration.',
                                           'Failures are retained; three successful resets do not establish a statistical success rate.',
                                           'Model turn counts come from actual SDK sessions; core metering rows were absent for this provider.']}
a.output.write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({k:summary[k] for k in ['successful_full_resets','order_variation_passed','maximum_position_error_m']}))
