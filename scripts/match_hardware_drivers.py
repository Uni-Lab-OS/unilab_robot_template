"""Maintain exact module driver candidates without claiming hardware support.

Reads explicitly supplied maintenance sources. Runtime only reads the resulting
brand definitions and explicitly installed dependencies, never this source tree.
"""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
from import_brand_catalogs import brand_key

ROOT=Path(__file__).resolve().parents[1]
DOC='https://robodk.com/doc/en/Robot-Drivers.html'
PROTOCOL='https://robodk.com/doc/en/PythonAPI/driver.html'
# File names are evidence selectors, not sufficient proof of model support.
FAMILIES={
 'abb':['ABBDriver.exe','apiabb.exe'], 'fanuc':['FanucSM.exe','FanucPCDK.exe'],
 'kuka':['kukabridge.exe','apikuka.exe','apikukaiiwa.py'],
 'dobot':['DobotV3Driver.py','DobotV4Driver.py','DobotMG400Driver.py','DobotDriver.exe'],
 'elite':['EliteRobotsDriverCS.py','EliteRobotsDriverEC.py'],
 'yaskawa':['MotomanHSE.exe','apimotoman.exe'],'kawasaki':['KawasakiDriver.py'],
 'comau':['apicomau.py'],'mitsubishi':['apimitsubishi.py'],'staubli':['apistaubli.exe'],
 'techmanrobot':['TMDriver.py'],'omron':['TMDriver.py'],'epson':['EpsonDriver.py'],
 'aubo':['DriverAubo.py','DriverAuboArcs.py'],'jaka':['JakaDriver.py','apijaka.py'],
 'denso':['apidenso.exe'],'universal_robots':['DriverUR.exe','apiur.exe'],
 'huayanhans':['HansDriver.py'],'hansrobot':['HansDriver.py'],
 'schneiderelectric':['SchneiderDriver.py'],'fairino':['FairinoDriver.py'],
 'ufactory':['xArmDriver.py'],'hanwha':['HanwhaDriver.py'],'kinova':['apikinova.py'],
 'borunte':['BorunteDriver.exe'],'annin':['AnninDriver.py','AR3Driver.py','AR2Driver.py'],
 'mecademic':['apimecademic.py','apimecademicpy.py'],'niryo':['NiryoDriver.py'],
 'wlkata':['WlkataDriver.py'],'productiverobotics':['ProductiveDriver.py'],
 'automata':['AutomataDriver.py'],'doosan':['DoosanDriver.exe'],
}
POST_ALIASES={'universal_robots':'Universal_Robots','huayanhans':'Hans','techmanrobot':'Techman',
 'yaskawa':'Motoman','schneiderelectric':'Schneider','mabirobotic':'Mabi', 'hansrobot':'Hans'}
SDK_REPOSITORIES={
 'abb':['libegm','librws','ros2'], 'elite':['cs_sdk','python_sdk','ros2_driver'],
 'dobot':['python_v4','ros2_v4'], 'fanuc':['ros2_driver','ros_industrial','ros_industrial_melodic_source'],
 'kuka':['ros2_drivers','lbr_fri_ros2','lbr_fri_ros2_rolling_source'],
 'universal_robots':['ur_rtde','ros2_driver','client_library','rtde_python'],
 'franka':['libfranka','franka_ros2','franka_ros_legacy'], 'aubo':['ros2_driver','ros_driver'],
 'realman':['api2','ros2'],'jaka':['python_sdk','cpp_sdk','ros2'],
 'agilebot':['ros2'],'annin':['ar4_ros2'],'automata':['eva_python_sdk'],
 'comau':['crcopen_ros2'],'denso':['ros2'],'doosan':['ros2','drfl'],
 'elephant':['python_sdk','mycobot_ros','mycobot_ros2'],'epson':['ros2'],
 'estun':['ros2_driver'],'fairino':['python_sdk','cpp_sdk','ros2'],
 'flexiv':['rdk','ros2'],'hiwin':['ros2'],'igus':['irc_ros2'],'inovo':['ros'],
 'kassowrobots':['kord_driver','ros2'],'kawasaki':['ros','ros2'],
 'mecademic':['python_api','ros2'],'mitsubishi':['ros2'],'nachi':['rosweld_drivers'],
 'neuromeka':['python_sdk','ros2'],'niryo':['python_sdk','ned_ros','ros2_bridge'],
 'omron':['tm_ros2'],'techmanrobot':['ros2_legacy','ros2_s_series'],
 'rainbowrobotics':['ros2'],'rokae':['cpp_sdk','python_sdk','ros2'],
 'siasun':['ros'],'standardbots':['ros2'],'staubli':['val3_driver','ros2'],
 'ufactory':['python_sdk','cpp_sdk','ros2','xarm_ros2_master_source'],
 'wlkata':['python_sdk','ros2'],'yaskawa':['ros2','ros_industrial'],
 'agilex':['piper_ros'],'kinova':['ros2_kortex','ros_kortex','ros2_kortex_main_source'],
 'robotiq':['ros2_gripper'],
}
SDK_BRAND={'kassowrobots':'kassow','techmanrobot':'techman','rainbowrobotics':'rainbow','standardbots':'standard_bots'}
PRIORITY={'dobot':'cr5','elite':'ec66','abb':'irb1200:5:0.9','fanuc':'lrmate200ic','kuka':'kr6r900sixx'}

def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def write(path,value):
 path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def driver_scope(brand,model,filename):
 """Return explicit controller candidates and exclusions, never guessed firmware."""
 if brand=='dobot':
  if filename=='DobotV3Driver.py': return (['Dobot V3'] if re.match(r'^(cr\d+(?!a)|nova)',model) and not re.match(r'^cr\d+a',model) else [],'CR/Nova V3 family; controller generation must be confirmed')
  if filename=='DobotV4Driver.py': return (['Dobot V4'] if re.match(r'^cr\d+a',model) or model in ('me6','magiciane6','e6') else [],'CRA/Magician E6 V4 family; do not apply to CR V3')
  if filename=='DobotMG400Driver.py':return (['Dobot MG400/M1 Pro'] if model in ('mg400','m1pro') else [],'MG400/M1 Pro only')
  return (['Dobot Magician'] if model=='magician' else [],'Magician controller only')
 if brand=='elite':return ([('Elite CS' if 'CS.py' in filename else 'Elite EC')] if model.startswith('cs' if 'CS.py' in filename else 'ec') else [],'EC and CS protocols are separate')
 if brand=='kuka':
  iiwa='iiwa' in model
  return ((['KUKA Sunrise IIWA'] if iiwa else []) if 'iiwa' in filename else ([] if iiwa else ['KUKA KRC2','KUKA KRC4']), 'KRC/IIWA controller families are distinct; install matching controller-side bridge')
 if brand in ('omron','techmanrobot'):
  return (['Techman TM'] if model.startswith('tm') or (brand=='techmanrobot' and model.startswith('techmantm')) else [],'TM driver does not establish support for Omron Adept or LD mobile robots')
 if brand=='abb':return ['ABB IRC5/RAPID socket'],'PC Interface socket option plus matching RDK_DriverSocket.mod; controller generation/firmware unverified'
 if brand=='fanuc':return (['FANUC PCDK'] if 'PCDK' in filename else ['FANUC Socket Messaging']), ('PCDK option required' if 'PCDK' in filename else 'R636/R648 socket messaging, R641 PC Interface, R632 KAREL and matching RDK_S3/GO_MJ programs; UTOOL/UFRAME 1 and registers 50..52 affected')
 return [brand+' controller (exact generation unresolved)'],'Brand-family file only; controller, firmware, options, model compatibility require deployment evidence'

def sdk_scope(brand,model):
 if brand=='dobot': return bool(re.match(r'^cr\d+a',model) or model in ('me6','magiciane6','e6')),'V4 CRA/E6 family; CR5 V3 is not covered by this SDK'
 if brand=='elite':return model.startswith('cs'),'CS only; controller 2.13.x >=2.13.4 or 2.14.x >=2.14.2; not EC'
 if brand=='omron':return model.startswith('tm'),'Techman TM only'
 return True,'SDK repository candidate; exact model/controller compatibility not established'

def main():
 p=argparse.ArgumentParser();p.add_argument('--source-inventory',type=Path,required=True);p.add_argument('--source-root',type=Path,required=True);p.add_argument('--robodk-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--robodk-manifest',type=Path);p.add_argument('--apply',action='store_true');a=p.parse_args()
 if a.output.resolve().is_relative_to(ROOT):raise ValueError('Evidence must be outside repository')
 inventory=json.loads(a.source_inventory.read_bytes());by_path={r['path'].replace('\\','/'):r for r in inventory};sources={};rdk={}
 for filename in sorted({f for fs in FAMILIES.values() for f in fs}):
  path=a.robodk_root/'api/Robot'/filename
  if not path.is_file():continue
  versions=[x.parent.name for x in path.parent.glob('v*/'+path.stem+'.pyc')]
  bytecode=[{'relative_path':x.relative_to(a.robodk_root).as_posix(),'sha256':sha(x)} for x in path.parent.glob('v*/'+path.stem+'.pyc')]
  rdk[filename]={'relative_path':path.relative_to(a.robodk_root).as_posix(),'sha256':sha(path),'bytes':path.stat().st_size,'compiled_python_versions':versions,'compiled_entrypoints':bytecode,'runtime_dependencies':{'driver_python':'one of compiled_python_versions' if versions else None,'native_runtime':'vendor SDK/DLL and controller-side programs may be required','complete_dependency_closure':'not_verified'},'license':'external_installation_only; redistribution_not_granted','source_url':DOC,'protocol_url':PROTOCOL}
 for brand,repos in SDK_REPOSITORIES.items():
  for repo in repos:
   key='robots/'+SDK_BRAND.get(brand,brand)+'/'+repo;row=by_path.get(key)
   if not row:continue
   path=a.source_root/key
   proof=[]
   for rel in row['readmes'][:2]+row['licenses']:
    f=path/rel
    if f.is_file():proof.append({'path':rel.replace('\\','/'),'sha256':sha(f)})
   code=[]
   if path.exists():
    for f in path.rglob('*'):
     if f.is_file() and f.suffix.lower() in ('.py','.cpp','.h','.hpp','.c','.so','.dll','.pyd') and '.git' not in f.parts:
      code.append({'path':f.relative_to(path).as_posix(),'sha256':sha(f)})
   sources[key]={'repository_url':row['url'],'source_path':key,'files':row['files'],'code_files':len(code),'code_tree_sha256':hashlib.sha256(json.dumps(code,sort_keys=True).encode()).hexdigest(),'files_evidence':proof,'code_manifest':code,'local_availability':'present' if code else 'empty_or_no_driver_code','license_status':'files_preserved_not_blanket_redistribution_approval','license_files':row['licenses']}
 reports=[];counter=Counter();brand_counts=defaultdict(Counter)
 adapter_hash=sha(ROOT/'packages/unilab-robot-runtime/src/unilab_robot_runtime/hardware_driver.py')
 for cp in sorted((ROOT/'packages').glob('*/src/*/catalog.json')):
  cat=json.loads(cp.read_bytes());brand=cat['brand']
  for model,entry in cat['models'].items():
   mp=cp.parent/entry['module_path'];definition=json.loads(mp.read_bytes());roles=definition['asset_role'];candidates=[];rejected=[]
   if 'arm' in roles:
    for filename in FAMILIES.get(brand,[]):
     controllers,restriction=driver_scope(brand,model,filename)
     if filename not in rdk:rejected.append({'source':filename,'reason':'driver_file_missing'});continue
     if not controllers:rejected.append({'source':filename,'reason':restriction});continue
     doc=DOC
     if brand in ('abb','fanuc','kuka','dobot'):doc='https://robodk.com/doc/en/Robots-'+{'abb':'ABB','fanuc':'Fanuc','kuka':'KUKA','dobot':'Dobot'}[brand]+'.html'
     candidates.append({'id':'robodk:'+Path(filename).stem,'kind':'external_online_driver','adapter':'robodk_stdio',**rdk[filename], 'controller_choices':controllers,'constraints':[restriction,'Exact controller firmware, installed options and calibrated model mapping are not inferred'],'model_compatibility':'family_candidate_only','documentation':doc,'operations':['connect','read_joints','joint_move_with_explicit_driver_pose'],'qualification':{'file_evidence':'present','family_match':'candidate','model_controller':'not_verified','offline_adapter':'not_run','live_connection':'not_run','hardware_execution':'not_run'}})
   use_sdk,reason=sdk_scope(brand,model)
   for repo in SDK_REPOSITORIES.get(brand,[]):
    key='robots/'+SDK_BRAND.get(brand,brand)+'/'+repo
    source=sources.get(key)
    if not source or source['local_availability']!='present':
     rejected.append({'source':key,'reason':'empty_or_no_driver_code'});continue
    if not use_sdk or ('tool' in roles and brand!='robotiq'):
     rejected.append({'source':key,'reason':reason if not use_sdk else 'arm_driver_not_tool_driver'});continue
    candidates.append({'id':'sdk:'+brand+':'+repo,'kind':'sdk_or_ros_driver_candidate','adapter':None,'source_ref':key,'repository_url':source['repository_url'],'code_tree_sha256':source['code_tree_sha256'],'source_evidence':source['files_evidence'],'constraints':[reason,'Runtime requires separately installed SDK/ROS dependencies; no upstream checkout lookup'],'license_status':source['license_status'],'qualification':{'file_evidence':'present','family_match':'candidate','model_controller':'not_verified','offline_adapter':'not_run','live_connection':'not_run','hardware_execution':'not_run'}})
   token=POST_ALIASES.get(brand,brand).lower();posts=[x.name for x in (a.robodk_root/'Posts').glob('*.py') if x.stem.lower()==token or x.stem.lower().startswith(token+'_')]
   rr=any(c['kind']=='external_online_driver' for c in candidates);ss=any(c['kind']=='sdk_or_ros_driver_candidate' for c in candidates)
   classification='both' if rr and ss else 'robodk_only' if rr else 'sdk_only' if ss else 'post_only' if posts and 'arm' in roles else 'unmatched'
   counter[classification]+=1;brand_counts[brand][classification]+=1
   previous=definition.get('drivers',{})
   # Keep previously verified adapters if source identity did not change.
   old={c['id']:c for c in previous.get('hardware_candidates',[])}
   for prior in old.values():
    if prior.get('kind')=='native_sdk_adapter':
     native=dict(prior)
     native['qualification']=dict(prior.get('qualification',{}))
     native_hash=sha(ROOT/'packages/unilab-robot-runtime/src/unilab_robot_runtime/native_sdk.py')
     current={c['id']:c for c in candidates}
     unchanged=all(s in current and old.get(s,{}).get('code_tree_sha256')==current[s].get('code_tree_sha256') for s in prior.get('source_candidates',[]))
     if not unchanged or prior.get('offline_evidence',{}).get('adapter_sha256')!=native_hash:
      native['qualification']['offline_adapter']='requires_revalidation'
     candidates.append(native)
   for c in candidates:
    prior=old.get(c['id'],{})
    if prior.get('sha256')==c.get('sha256') and prior.get('code_tree_sha256')==c.get('code_tree_sha256') and prior.get('qualification',{}).get('offline_adapter')=='passed' and (prior.get('offline_evidence') or {}).get('adapter_sha256')==adapter_hash:
     c['qualification']['offline_adapter']='passed';c['offline_evidence']=prior.get('offline_evidence')
   definition['drivers']={**previous,'hardware_candidates':candidates,'hardware_match_scope':classification,'hardware_match_reason':'Controller-family candidates do not grant model compatibility or execution capability','offline_posts':posts,'excluded_driver_sources':rejected}
   if a.apply:
    write(mp,definition);entry['module_sha256']=sha(mp)
   reports.append({'brand':brand,'model':model,'roles':roles,'asset_id':definition['asset']['asset_id'],'classification':classification,'candidates':candidates,'post_processors_only_evidence':posts,'excluded':rejected,'hardware_verified':False})
  if a.apply:write(cp,cat)
 write(a.output/'installed-robodk-drivers.json',rdk);write(a.output/'sdk-source-evidence.json',sources);write(a.output/'model-driver-matches.json',reports)
 summary={'as_of':datetime.now(timezone.utc).isoformat(),'models':len(reports),'classification':dict(counter),'by_brand':{b:dict(c) for b,c in sorted(brand_counts.items())},'hardware_verified_models':0,'rule':'Candidates, never hardware compatibility. Posts excluded from online coverage. Empty repositories excluded from SDK code coverage.','applied':a.apply}
 if a.robodk_manifest:
  manifest=json.loads(a.robodk_manifest.read_bytes());comparison=[]
  for category in manifest['categories']:
   for item in category.get('items',[]):
    if item.get('category')!='robots':continue
    meta=item['metadata'];raw=meta.get('B') or 'unknown';normalized=re.sub(r'[^a-z0-9_]','',raw.lower())
    normalized={'yaskawamotoman':'yaskawa','anninrobotics':'annin'}.get(normalized,normalized)
    b=brand_key(normalized);m=re.sub(r'[^a-z0-9]','',(meta.get('M') or '').lower())
    available=[f for f in FAMILIES.get(b,[]) if f in rdk]
    scoped=[f for f in available if driver_scope(b,m,f)[0]]
    comparison.append({'source_id':item['sourceId'],'brand':b,'model':meta.get('M'),'broad_family_file_match':bool(available),'series_filtered_family_candidate':bool(scoped),'drivers':scoped,'hardware_verified':False})
  write(a.output/'robodk-1468-comparison.json',comparison)
  summary['robodk_source_comparison']={'total':len(comparison),'broad_family_file_match':sum(x['broad_family_file_match'] for x in comparison),'series_filtered_family_candidate':sum(x['series_filtered_family_candidate'] for x in comparison),'prior_user_report':964,'scope':'same raw RoboDK robot entries, current Windows installation; no live support claim'}
 write(a.output/'matching-summary.json',summary);print(json.dumps({k:v for k,v in summary.items() if k!='by_brand'},ensure_ascii=False))

if __name__=='__main__':main()
