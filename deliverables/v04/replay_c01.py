"""Replay public C01 evidence without its deliberately excluded private mission DB."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
from patrol import verify_visit


def replay(root):
    read = lambda p: json.loads(p.read_text())
    config = read(root / 'execution_config.json')
    if config.get('require_obstacle_evidence'):
        raise ValueError('This public adapter supports the standard C01 compatibility task only')
    final = read(root / 'actions/patrol.verification.json')
    canonical = read(root / 'canonical-receipts-final.json')
    receipts = {r['receipt']['action_id']: r['receipt'] for r in canonical if r.get('receipt')}
    failures = []
    visits = []
    for visit in final['visits']:
        artifact = visit['artifact']
        path = root / 'actions' / Path(artifact['path']).name
        if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != artifact['sha256']:
            raise ValueError('C01 artifact SHA mismatch')
        data = read(path)
        receipt = receipts.get(data['action_id'], {})
        valid = (
            receipt.get('capability_id') == 'navigation.navigate_to_pose'
            and receipt.get('body_id') == config['body_id']
            and receipt.get('body_snapshot_hash') == config['body_snapshot_hash']
            and receipt.get('final_state') == 'COMPLETED'
            and receipt.get('evidence_domain') == 'SIMULATION'
            and receipt.get('verification_result', {}).get('evidence_artifact') == artifact
        )
        proof = verify_visit(data['trajectory'], config['sites'][data['site_id']], data['nav2'], data['verification']['sensor'], body_path=config['physics_body_path'])
        if not valid or not proof['success']:
            failures.append('C01 canonical binding or frozen physical verification failed')
        visits.append({'site_id': data['site_id'], 'verification': proof, 'artifact_sha256': artifact['sha256']})
    if [v['site_id'] for v in visits] != config['expected_order']:
        failures.append('C01 task visit order mismatch')
    memory = [r for r in receipts.values() if r.get('capability_id') == 'patrol.verify_and_remember' and r.get('final_state') == 'COMPLETED']
    if not memory or final.get('memory_outcome') != 'success' or final.get('success') is not True:
        failures.append('C01 successful Memory receipt missing')
    for r in memory:
        if r.get('body_id') != config['body_id'] or r.get('body_snapshot_hash') != config['body_snapshot_hash'] or r.get('evidence_domain') != 'SIMULATION':
            failures.append('C01 Memory binding mismatch')
    task = read(root / 'task-kernel.json')
    if task.get('state') != 'SUCCEEDED' or (memory and datetime.fromisoformat(task['updated_at']) < max(datetime.fromisoformat(r['finished_at']) for r in memory)):
        failures.append('C01 TaskKernel/Memory ordering mismatch')
    return {'status': 'FAIL' if failures else 'PASS', 'failures': failures, 'visits': visits, 'task_kernel': task, 'scope': 'Public archive adapter calls the original frozen verify_visit and validates original canonical receipts, hashes, visit order and exported actual TaskKernel/Memory timestamps. No private DB or journal is synthesized; archived code and receipts are unchanged. Artifact bytes resolve by verified basename while receipt bindings retain their original paths.'}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    result = replay(a.directory.resolve())
    a.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'status': result['status'], 'visits': len(result['visits'])}))
    raise SystemExit(0 if result['status'] == 'PASS' else 1)
