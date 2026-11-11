"""Five exact brand bindings through an offline RoboDK protocol peer.

This does NOT run the proprietary driver or any robot. It verifies our installed
adapter's command/response boundary against explicit documented transcripts.
"""
import argparse
import hashlib
import importlib
import json
import math
from pathlib import Path
import sys

from unilab_robot_runtime.hardware_driver import ControllerJointMap, LineProcess, DriverProtocolError, DriverTimeout
import unilab_robot_runtime.hardware_driver as driver_module

ROOT=Path(__file__).resolve().parents[1]
CASES=[('dobot','cr5','DobotV3Driver','Dobot V3'),('elite','ec66','EliteRobotsDriverEC','Elite EC'),
 ('abb','irb1200:5:0.9','ABBDriver','ABB IRC5/RAPID socket'),('fanuc','lrmate200ic','FanucSM','FANUC Socket Messaging'),
 ('kuka','kr6r900sixx','kukabridge','KUKA KRC4')]

def write(p,d):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True,type=Path);p.add_argument('--record',action='store_true');p.add_argument('--peer',type=Path,default=ROOT/'tests/fixtures/robodk_stdio_peer.py');a=p.parse_args()
 if a.output.resolve().is_relative_to(ROOT):raise ValueError('Evidence must be outside repo')
 results=[]
 # Synthetic identity mapping with nonzero coordinates. This does not certify
 # controller-to-CAD calibration; affine/coupled mappings have separate tests.
 m=ControllerJointMap(tuple(tuple(float(i==j) for j in range(6)) for i in range(6)),(0.,)*6,(-math.pi,)*6,(math.pi,)*6,'offline synthetic fixture, NOT hardware calibration')
 q=tuple(math.radians(x) for x in (10,-20,30,-40,50,-60))
 for brand,model,driver,controller in CASES:
  module=importlib.import_module('unilab_robot_'+brand);definition=module.module_definition(model)
  out=a.output/brand;out.mkdir(parents=True,exist_ok=True)
  common=[{'request':'CONNECT offline.invalid 2000 6','responses':['SMS:Working...','SMS:Ready']},
          {'request':'CJNT','responses':['JNTS 0,0,0,0,0,0','SMS:Ready']}]
  move={'request':'MOVJ 10 -20 30 -40 50 -60 100 200 300 0 90 180','responses':['SMS:Working...','JNTS_MOVING 5,-10,15,-20,25,-30','SMS:Ready']}
  tests={
   'success':common+[move,{'request':'CJNT','responses':['JNTS 10,-20,30,-40,50,-60','SMS:Ready']},{'request':'DISCONNECT','responses':['SMS:Disconnected']}],
   'controller_error':common+[{'request':move['request'],'responses':['SMS:Working...','SMS:Error controller rejected command']}],
   'timeout':common+[{'request':move['request'],'responses':['SMS:Working...'],'delay':2}],
   'target_mismatch':common+[move,{'request':'CJNT','responses':['JNTS 0,0,0,0,0,0','SMS:Ready']}],
  }
  statuses={}
  for label,scenario in tests.items():
   scenario_path=out/(label+'-scenario.json');write(scenario_path,scenario)
   process=LineProcess([sys.executable,'-B','-u',str(a.peer.resolve()),str(scenario_path.resolve())],cwd=out,timeout_s=.5)
   adapter=module.create_hardware_driver(model,'robodk:'+driver,mapping=m,controller=controller,
     deployment_evidence='offline fixture; no deployment qualification',offline_process=process,
     targets={'fixture_target':(q,(100,200,300,0,90,180))})
   try:
    initial=adapter.connect('offline.invalid',2000);assert all(abs(v)<1e-10 for v in initial)
    try:
     receipt=adapter.execute_target('fixture_target',{'resolved_target':{'type':'joint_positions','joint_positions_si':list(q)}},'offline-001')
     assert label=='success','Unexpected success for '+label
     assert receipt['completed'] and receipt['hardware_verified'] is False and receipt['scope']=='offline_protocol_fixture'
     assert max(abs(x-y) for x,y in zip(receipt['joint_positions_si'],q))<1e-10
     # Reject stale command retries and unallowlisted PointSet targets.
     try:adapter.execute_target('fixture_target',{'resolved_target':{'type':'joint_positions','joint_positions_si':list(q)}},'offline-001')
     except ValueError:pass
     else:raise AssertionError('Duplicate command was replayed')
     assert adapter.query_command('offline-001') is None
     assert adapter.request_stop('offline-001','fixture') is False
     write(out/'completion-receipt.json',receipt)
    except DriverProtocolError as exc:
     assert label!='success',str(exc)
     if label=='timeout':assert isinstance(exc,DriverTimeout)
     assert adapter.process.failed
     assert adapter.query_command('offline-001') is None
     statuses[label]={'status':'expected_rejection','exception':type(exc).__name__,'message':str(exc)}
    else:statuses[label]={'status':'passed'}
   finally:
    adapter.close();write(out/(label+'-transcript.json'),process.transcript)
  receipt={'brand':brand,'model':model,'asset':definition['asset'],'driver_id':'robodk:'+driver,
   'controller_selection_for_fixture':controller,'status':'passed','qualification':'offline_protocol_adapter',
   'vendor_driver_executed':False,'hardware_connected':False,'hardware_motion':False,
   'joint_mapping':'synthetic identity; affine permutation/coupling tested separately; not commissioned',
   'peer_sha256':digest(a.peer),'adapter_sha256':digest(Path(driver_module.__file__)),'tests':statuses}
  write(out/'receipt.json',receipt);results.append(receipt)
  if a.record:
   cat=module.catalog;catalog_path=cat.root/'catalog.json';c=json.loads(catalog_path.read_bytes());row=c['models'][model];mp=cat.root/row['module_path'];d=json.loads(mp.read_bytes())
   candidate=next(x for x in d['drivers']['hardware_candidates'] if x['id']=='robodk:'+driver)
   candidate['qualification']['offline_adapter']='passed'
   candidate['offline_evidence']={'scope':'separate-process protocol fixture; vendor driver not executed; synthetic coordinates', 'adapter_sha256':receipt['adapter_sha256'],'receipt_sha256':digest(out/'receipt.json'),'tests':list(tests),'hardware_verified':False}
   write(mp,d);row['module_sha256']=digest(mp);write(catalog_path,c)
 write(a.output/'summary.json',{'status':'passed','models':len(results),'scope':'offline protocol adapters only','hardware_verified':0,'results':results})
 print(json.dumps({'status':'passed','offline_adapter_models':len(results),'hardware_verified':0}))

if __name__=='__main__':main()
