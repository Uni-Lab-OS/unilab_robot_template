"""Attach native SDK entries to the original SDK-only cohort, without granting hardware capability."""
import argparse
import hashlib
import json
from pathlib import Path
from collections import Counter

ROOT=Path(__file__).resolve().parents[1]
RULES={
 'agilebot':('agilebot_a','Agilebot A controller / TP proxy 5609','Agilebot.Robot.SDK.A 2.0.1.0','agilebot_a-receipt.json',['connect','read_joints','move_joint_request'],'GBT-P7A controller/firmware must be confirmed; loopback model response is a fixture'),
 'elephant':('elephant280','myCobot280 M5 TCP bridge','pymycobot 4.0.4','elephant280-receipt.json',['connect','read_joints','move_joint_request'],'280 M5 requires configured TCP bridge; no automatic Pro600/Pro630 equivalence'),
 'igus':('igus_cri','igus Robot Control CRI','CRI-Python-Lib pinned source','igus_cri-receipt.json',['connect','read_joints','move_joint_request'],'iRC model configuration and axes count require deployment confirmation'),
 'estun':('estun_eri','ESTUN ERI','ERI SDK 1.0.0.17','estun-native/receipt.json',['connect','read_joints','movj_request_requires_native_units_and_control_evidence'],'Linux SCHED_RR priority 51 blocked EPERM; exact controller/firmware and joint units remain unverified'),
 'hiwin':('hiwin_commander','HIWIN HRSDK command socket','hiwin_robot_client_library pinned source + explicit transport correction','hiwin-native/factory-receipt.json',['connect','read_joints','move_joint_request'],'Official RA6/RS4 scope; RT605 compatibility not established; native SDK transport bug corrected transparently'),
 'inovo':('inovo_commander','Inovo ROS Noetic Commander','commander_api pinned source','inovo-native/factory-receipt.json',['connect','read_joints','move_cartesian_action'],'ROS1 Noetic messages/action server required; no joint-target API in this SDK; external Python3.8 worker'),
 'flexiv':('flexiv_rdk_1_9','Flexiv RDK server 1.9-compatible firmware','flexivrdk 1.9.4.1','flexiv-native/receipt.json',['connect','read_joints','joint_position_request'],'RDK2.1 rejects Rizon4s; 1.9.4 actual constructor executed but no licensed robot RDK server; read/motion unvalidated'),
 'franka':('franka_panda_0_9','Franka Panda FCI / libfranka 0.9.2','libfranka 0.9.2; Panda system >=4.2.1','franka-panda/factory-receipt.json',['connect','read_joints'],'Correct Panda SDK generation; synthetic FCI loopback validates read boundary only; realtime motion deliberately unavailable; modern pylibfranka 0.21 is not Panda-compatible'),
}

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,obj):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def main():
 p=argparse.ArgumentParser();p.add_argument('--cohort',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--apply',action='store_true');a=p.parse_args()
 if a.output.resolve().is_relative_to(ROOT):raise ValueError('Evidence must be external')
 source=json.loads(a.cohort.read_bytes());cohort=[r for r in source if r['classification']=='sdk_only']
 if len(cohort)!=53:raise ValueError('Keep the original 53-model cohort traceable; unexpected input cohort')
 lookup={(r['brand'],r['model']):r for r in cohort};rows=[]
 code=ROOT/'packages/unilab-robot-runtime/src/unilab_robot_runtime/native_sdk.py';digest=sha(code)
 for cp in (ROOT/'packages').glob('*/src/*/catalog.json'):
  cat=json.loads(cp.read_bytes());changed=False
  for model,entry in cat['models'].items():
   key=(cat['brand'],model)
   if key not in lookup:continue
   original=lookup[key];family,controller,version,evidence,operations,reason=RULES[key[0]]
   mp=cp.parent/entry['module_path'];definition=json.loads(mp.read_bytes())
   axis_types=definition.get('motion_model',{}).get('parameters',{}).get('robot_parameters',{}).get('joint_types',[])
   blocked_model=False
   if key==('elephant','mycobotpro600'):
    family='elephant_pro600_local';controller='Pro600 Phoenix controller-local LinuxCNC';evidence='elephant_pro600_local-receipt.json';operations=['connect_local','read_joints']
    reason='SDK explicitly declares Robots.PRO_600=502; Phoenix requires Debian aarch64 LinuxCNC/HAL/CAN runtime absent here; readonly do_init=False never configures or powers hardware'
   proof=a.evidence/evidence
   result=json.loads(proof.read_bytes()) if proof.is_file() else {}
   passed=result.get('status') in ('passed','vendor_calls_exercised','passed_read_only_sdk_boundary') and (result.get('worker_sha256') or result.get('adapter_sha256'))==digest and key[0] in ('agilebot','elephant','igus','hiwin','inovo','franka') and not blocked_model
   record={'id':'native:'+family,'kind':'native_sdk_adapter','adapter':'vendor_native' if not blocked_model else None,
    'native_family':family,'controller_choices':[controller],'sdk_version_requirement':version,
    'source_candidates':[c['id'] for c in original['candidates']],'operations':operations,
    'constraints':[reason,'SDK/model deployment calibration is explicit; no assets or upstream source checkout required at runtime','Mixed axes require explicit per-axis types/units and source evidence; generic unit support is not model qualification'],
    'qualification':{'implementation':'implemented' if not blocked_model else 'blocked_model_scope','model_controller':'not_verified',
     'axis_types':'mixed_prismatic_vendor_units_not_verified' if 'prismatic' in axis_types else 'source_rotary_declared_deployment_mapping_required' if axis_types and all(t=='revolute' for t in axis_types) else 'axis_types_unknown_explicit_deployment_declaration_required',
     'offline_adapter':'vendor_family_boundary_exercised' if passed else 'blocked',
     'live_connection':'not_run','hardware_execution':'not_run'},
    'offline_evidence':{'receipt':evidence,'receipt_sha256':sha(proof) if proof.is_file() else None,'adapter_sha256':digest,
     'scope':'vendor SDK family boundary; this model was not individually connected'},
    'redistribution':'External SDK installation; no vendor SDK binaries included in brand package'}
   rows.append({'brand':key[0],'model':model,'original_classification':'sdk_only','original_sources':original['candidates'],**record})
   candidates=definition.setdefault('drivers',{}).setdefault('hardware_candidates',[])
   definition['drivers']['hardware_candidates']=[c for c in candidates if not c['id'].startswith('native:')]+[record]
   if a.apply:write(mp,definition);entry['module_sha256']=sha(mp);changed=True
  if changed:write(cp,cat)
 if len(rows)!=53:raise ValueError('Not all original models found in current brand catalog')
 write(a.output/'native-sdk-53.json',rows)
 summary={'original_cohort':53,'by_brand':dict(Counter(r['brand'] for r in rows)),
  'implemented_entries':sum(r['adapter']=='vendor_native' for r in rows),'family_offline_exercised_models':sum(r['qualification']['offline_adapter']=='vendor_family_boundary_exercised' for r in rows),
  'real_hardware_verified':0,'remaining':[{k:r[k] for k in ('brand','model','constraints')} for r in rows if r['qualification']['offline_adapter']=='blocked'],'applied':a.apply}
 write(a.output/'native-sdk-summary.json',summary);print(json.dumps(summary,ensure_ascii=False))

if __name__=='__main__':main()
