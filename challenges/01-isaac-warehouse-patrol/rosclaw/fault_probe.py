"""Real single-goal adapter fault test, separate from Native mission acceptance."""
import argparse
from datetime import datetime, timedelta, timezone
import json
import math
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'evaluator'))
from point_executor import PointExecutor, ScanWitness
from rosclaw.connectors.ros.action_client import Ros2ActionClient
from rosclaw.connectors.ros.transport.base import RosbridgeEndpoint
from rosclaw.connectors.ros.transport.rosbridge import RosbridgeTransport
from rosclaw.kernel import ExecutionMode


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--directory',type=Path,required=True)
    p.add_argument('--fault',choices=['disconnect','timeout'],required=True)
    p.add_argument('--site',default='entry')
    a=p.parse_args()
    config=json.loads(a.config.read_text())
    a.directory.mkdir(parents=True,exist_ok=False)
    transports=[RosbridgeTransport(RosbridgeEndpoint.from_url('ws://127.0.0.1:19091')) for _ in range(2)]
    for t in transports:
        if not t.connect().ok:
            raise RuntimeError('transport did not connect')
    client=Ros2ActionClient(transports[0]);sensor=ScanWitness(transports[1])
    deadline=time.monotonic()+5
    while not sensor.snapshot() and time.monotonic()<deadline:
        time.sleep(.1)
    executor=PointExecutor(root=a.directory,config=config,client=client,sensor=sensor)
    action=SimpleNamespace(execution_mode=ExecutionMode.SIMULATION,body_id=config['body_id'],
                           body_snapshot_hash=config['body_snapshot_hash'],capability_id='navigation.navigate_to_pose',
                           action_id='adapter-fault-'+a.directory.name,arguments={'site_id':a.site},
                           deadline_at=datetime.now(timezone.utc)+timedelta(seconds=4 if a.fault=='timeout' else 40))
    def disconnect():
        time.sleep(3)
        for t in transports:
            t.close()
    if a.fault=='disconnect':
        threading.Thread(target=disconnect,daemon=True).start()
    start=time.time()
    try:
        result=executor(action)
        artifacts=list((a.directory/'actions').glob('*.json'))
        evidence=json.loads(artifacts[0].read_text()) if artifacts else {}
        report={'fault':a.fault,'scope':'real adapter single-goal negative test, not Native mission',
                'wall_seconds':time.time()-start,'final_state':result.final_state.value,
                'physical_stop_verified':evidence.get('physical_stop_verified',False),
                'peak_linear_speed_mps':max((math.hypot(*s['linear_velocity_xyz'][:2]) for s in evidence.get('trajectory',[])),default=0),
                'peak_angular_speed_rps':max((abs(s['angular_velocity_xyz'][2]) for s in evidence.get('trajectory',[])),default=0),
                'feedback_count':sum(e['event']=='feedback' for e in evidence.get('events',[])),
                'error':evidence.get('error'),'nav2':evidence.get('nav2'),
                'DDS_fallback_events':[e for e in evidence.get('events',[]) if e['event']=='DDS_cancel_fallback'],
                'status':'PENDING_CANCEL_RECEIPT'}
        cancel_confirmed = evidence.get('nav2',{}).get('status') == 5
        if a.fault == 'disconnect':
            cancel_confirmed = any(
                e['returncode'] == 0 and json.loads(e['stdout']).get('acknowledged')
                for e in report['DDS_fallback_events']
            )
        report['cancel_confirmed'] = cancel_confirmed
        report['status'] = 'PASS' if result.final_state.value == 'FAILED' and evidence.get('physical_stop_verified') and cancel_confirmed else 'FAIL'
        (a.directory/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report))
    finally:
        sensor.close();client.close()


if __name__=='__main__':
    main()
