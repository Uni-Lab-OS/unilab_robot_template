"""Record genuine installed-driver evidence independently of synthetic protocol tests."""
import argparse,hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
CASES=[('dobot','cr5','DobotV3Driver.py'),('elite','ec66','EliteRobotsDriverEC.py'),
 ('abb','irb1200:5:0.9','ABBDriver.exe'),('fanuc','lrmate200ic','FanucSM.exe'),('kuka','kr6r900sixx','kukabridge.exe')]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def main():
 p=argparse.ArgumentParser();p.add_argument('--evidence',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();rows=[]
 adapter=ROOT/'packages/unilab-robot-runtime/src/unilab_robot_runtime/hardware_driver.py'
 for brand,model,driver in CASES:
  proof=a.evidence/(driver+'-adapter-receipt.json');receipt=json.loads(proof.read_bytes())
  if receipt.get('adapter_sha256')!=sha(adapter):raise ValueError('Actual driver evidence requires current adapter code digest')
  cp=ROOT/f'packages/unilab-robot-{brand}/src/unilab_robot_{brand}/catalog.json';cat=json.loads(cp.read_bytes());entry=cat['models'][model];mp=cp.parent/entry['module_path'];definition=json.loads(mp.read_bytes())
  selected=[c for c in definition['drivers']['hardware_candidates'] if c['id']=='robodk:'+Path(driver).stem]
  if len(selected)!=1 or selected[0]['sha256']!=receipt['sha256']:raise ValueError('Driver evidence does not match exact candidate')
  candidate=selected[0];candidate['qualification']['offline_adapter']='passed' if receipt['status']=='passed' else 'blocked'
  candidate['qualification']['vendor_process']='exercised'
  candidate['offline_evidence']={'receipt':proof.name,'receipt_sha256':sha(proof),'adapter_sha256':sha(adapter),'scope':receipt['scope'],
   'status':receipt['status'],'remaining':receipt.get('error'),'hardware_verified':False}
  if driver=='kukabridge.exe':candidate['feedback_layout']={'count':12,'arm_indices':[0,1,2,3,4,5],'scope':'six-axis KRC controlled peer; external E1..E6 are not commanded by this adapter'}
  if driver=='DobotV3Driver.py':candidate['connect_side_effects']=['Installed V3 driver sends EnableRobot() on dashboard port 29999; explicit deployment authorization required before physical use']
  if brand=='elite':
   native=a.evidence/'elite_ec-receipt.json';sdk=json.loads(native.read_bytes()) if native.is_file() else {}
   native_code=ROOT/'packages/unilab-robot-runtime/src/unilab_robot_runtime/native_sdk.py'
   passed=sdk.get('status')=='vendor_calls_exercised' and sdk.get('adapter_sha256')==sha(native_code)
   ec={'id':'native:elite_ec','kind':'native_sdk_adapter','adapter':'vendor_native','native_family':'elite_ec',
    'controller_choices':['Elite EC controller >=3.0.0'],'sdk_version_requirement':'elirobots 0.0.7 (official ROS README dependency)',
    'operations':['connect','read_joints','move_joint_request'],'constraints':['Explicit external-axis hold values required; no automatic servo enable or stop qualification','SDK fixed JSONRPC id and single recv are upstream transport limitations; no physical execution qualification'],
    'qualification':{'implementation':'implemented','offline_adapter':'vendor_family_boundary_exercised' if passed else 'blocked','model_controller':'not_verified','live_connection':'not_run','hardware_execution':'not_run'},
    'offline_evidence':{'receipt':native.name,'receipt_sha256':sha(native) if native.is_file() else None,'adapter_sha256':sha(native_code)},
    'redistribution':'External elirobots installation; vendor files are not included in brand wheel'}
   definition['drivers']['hardware_candidates']=[c for c in definition['drivers']['hardware_candidates'] if c['id']!='native:elite_ec']+[ec]
  write(mp,definition);entry['module_sha256']=sha(mp);write(cp,cat)
  rows.append({'brand':brand,'model':model,'driver':driver,'status':receipt['status'],'receipt':str(proof),'error':receipt.get('error'),'hardware_verified':False})
  if brand=='elite':rows[-1]['native_sdk_status']='passed' if passed else 'blocked';rows[-1]['native_sdk_receipt']=str(native)
 write(a.output,{'models':rows,'robodk_closed_loop_passed':sum(r['status']=='passed' for r in rows),
  'vendor_closed_loop_passed':sum(r['status']=='passed' or r.get('native_sdk_status')=='passed' for r in rows),'hardware_verified':0});print(json.dumps(rows,ensure_ascii=False))
if __name__=='__main__':main()
