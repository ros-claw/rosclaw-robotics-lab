"""Offline independent replay of a whitelist evidence archive; no GPU/model needed."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import PurePosixPath
import zipfile
import tempfile
from pathlib import Path
from obstacle import verify_obstacle

from patrol import verify_visit


def replay(path):
    failures=[]
    with zipfile.ZipFile(path) as z:
        def read(name):
            return json.loads(z.read(name))
        manifest=read('public-evidence-manifest.json')
        for name, expected in manifest.items():
            if hashlib.sha256(z.read(name)).hexdigest()!=expected:
                failures.append('archive file hash mismatch: '+name)
        config=read('execution_config.json')
        source=read('source-freeze.json')
        for name, expected in source['source_hashes'].items():
            if hashlib.sha256(z.read('frozen-source/'+name)).hexdigest()!=expected:
                failures.append('frozen source hash mismatch: '+name)
        final=read('actions/patrol.verification.json')
        receipts=[x['receipt'] for x in read('canonical-receipts-final.json') if x.get('receipt')]
        canonical={r['action_id']:r for r in receipts}
        visits=[]
        trajectories=[]
        observer_ids=set()
        for visit in final['visits']:
            artifact=visit['artifact']
            name='actions/'+PurePosixPath(artifact['path']).name
            if hashlib.sha256(z.read(name)).hexdigest()!=artifact['sha256']:
                failures.append('visit artifact hash mismatch')
            data=read(name)
            receipt=canonical.get(data['action_id'],{})
            if (receipt.get('body_id')!=config['body_id']
                or receipt.get('body_snapshot_hash')!=config['body_snapshot_hash']
                or receipt.get('final_state')!='COMPLETED'
                or receipt.get('evidence_domain')!='SIMULATION'
                or receipt.get('verification_result',{}).get('evidence_artifact')!=artifact):
                failures.append('canonical visit binding mismatch')
            proof=verify_visit(data['trajectory'],config['sites'][data['site_id']],data['nav2'],data['verification']['sensor'],body_path=config['physics_body_path'])
            if not proof['success']:
                failures.append('physical verification failed: '+data['site_id'])
            trajectories.extend(data['trajectory'])
            observer_ids.update(s.get('observer_id') for s in data['trajectory'])
            visits.append({'site_id':data['site_id'],'position_error_m':proof['position_error_m'],'stable_dwell_sim_seconds':proof['stable_dwell_sim_seconds'],'collision_count':proof['collision_count']})
        if len(observer_ids)!=1 or [v['site_id'] for v in visits]!=config['expected_order']:
            failures.append('reset identity or operator order mismatch')
        obstacle_proof = None
        if config.get('require_obstacle_evidence'):
            with tempfile.TemporaryDirectory() as d:
                directory=Path(d)
                for name in ['scene-audit.json','path-witness.jsonl']:
                    (directory/name).write_bytes(z.read('independent/'+name))
                obstacle_proof=verify_obstacle(directory,trajectories,
                                              final['visits'][0]['started_wall_time'],
                                              final['visits'][-1]['finished_wall_time'],
                                              config.get('obstacle_static_map_validation'))
            if obstacle_proof['status']!='PASS' or obstacle_proof!=final.get('obstacle_verification'):
                failures.append('unmapped obstacle physical/witness binding failed')
        memory=[r for r in receipts if r.get('capability_id')=='patrol.verify_and_remember' and r.get('final_state')=='COMPLETED' and r.get('evidence_domain')=='SIMULATION']
        task=read('task-kernel.json')
        if not memory or final.get('memory_outcome')!='success':
            failures.append('successful verified Memory receipt missing')
        elif (task.get('state')!='SUCCEEDED' or datetime.fromisoformat(task['updated_at'])<datetime.fromisoformat(memory[-1]['finished_at'])):
            failures.append('TaskKernel closure missing or early')
        return {'run_id':config['mission_id'],'status':'FAIL' if failures else 'PASS','failures':failures,'visits':visits,'obstacle_verification':obstacle_proof,'scope':'offline receipt/hash/physical replay; does not independently reproduce live simulator or model'}


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('archive')
    a=p.parse_args()
    result=replay(a.archive)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['status']=='PASS' else 1)
