"""Apply explicit published canonical identities, keeping drivers and providers.

Unlike discovery imports, this command changes existing locks. Exact published
keys and an independently classified scope report are both required. Old
definitions and invalidated asset qualifications are retained outside the repo.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

from import_brand_catalogs import ROOT, write, digest
from unilab_robot_contracts._asset_package import verify, inside


def main():
 p=argparse.ArgumentParser();p.add_argument('--targets',type=Path,nargs='+',required=True);p.add_argument('--scope-report',type=Path,required=True);p.add_argument('--library',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if a.output.resolve().is_relative_to(ROOT):raise ValueError('Evidence must remain outside repository')
 scope={(r['brand'],r['model']):r['roles'] for r in json.loads(a.scope_report.read_bytes())['rows']}
 catalogs={}
 for cp in (ROOT/'packages').glob('*/src/*/catalog.json'):
  c=json.loads(cp.read_bytes());catalogs[c['brand']]=(cp,c)
 events=[];pending=[]
 # Validate complete scope before touching any source module. Older scope
 # snapshots may predate admitted brand aliases/models.
 for source in a.targets:
  for target in json.loads(source.read_bytes())['targets']:
   role=scope.get(tuple(target['key']))
   if not role or any(x not in ('arm','tool') for x in role):raise ValueError('Unclassified or out-of-scope model: '+str(target['key']))
 for source in a.targets:
  receipt=json.loads(source.read_bytes())
  active_mapping=receipt['status']=='verified_local_active_revision_mapping'
  if active_mapping:
   index_path=a.library/'library.json'
   if digest(index_path)!=receipt.get('active_index_sha256'):raise ValueError('Local active index changed since reconciliation audit')
   index=json.loads(index_path.read_bytes());active={}
   for record in index['records']:
    for target in record.get('targets',[]):
     if not target.get('member') and all(k in target for k in ('asset_id','package_digest','package_path')):
      active.setdefault(target['asset_id'],set()).add(tuple(target[k] for k in ('asset_id','package_digest','package_path')))
   for target in receipt['targets']:
    expected=tuple(target[k] for k in ('asset_id','package_digest','package_path'))
    if active.get(target['asset_id'])!={expected}:raise ValueError('Active revision must be exact and unambiguous')
  elif receipt['status']!='oss_published_local_installed':raise ValueError('Only formal publication/install or verified explicit local-active mappings accepted')
  for t in receipt['targets']:
   brand,model=t['key'];role=scope.get((brand,model))
   if not role or any(x not in ('arm','tool') for x in role):raise ValueError('Unclassified or out-of-scope model')
   cp,cat=catalogs[brand];binding={k:t[k] for k in ('asset_id','package_digest','package_path')};package=inside(a.library,binding['package_path'])
   verified=verify(package,binding['package_digest'])
   if verified['asset_id']!=binding['asset_id']:raise ValueError('Published identity mismatch')
   adapter=json.loads((package/'simready_usd/unilab.json').read_bytes())
   row=cat['models'].get(model)
   if row and row['asset']==binding:
    events.append({'brand':brand,'model':model,'status':'already_current'});continue
   if row:
    mp=cp.parent/row['module_path'];definition=json.loads(mp.read_bytes());old=definition['asset']
    write(a.output/'previous'/brand/mp.parent.name/'module.json',definition)
    write(a.output/'previous'/brand/mp.parent.name/'assets.lock.json',json.loads((mp.parent/'assets.lock.json').read_bytes()))
    obsolete=definition.get('planning',{}).pop('asset_adapter',None)
    if obsolete:definition.setdefault('planning',{})['asset_adapter_status']='revalidation_required_after_asset_revision'
    if 'asset_planning' in definition['capabilities']:definition['capabilities']['asset_planning']='revalidation_required_after_asset_revision'
    # Provider URDF identity is preserved, but never transferred to new USD.
    definition['asset']=binding;definition.setdefault('composition',{})['root_asset']=binding
    definition.setdefault('motion_model',{})['parameters']=adapter.get('parameters',{})
    definition['motion_model']['entrypoints']=adapter['entrypoints']
    definition['behavior']=adapter.get('behavior_binding',{})
    definition['source_provenance']['active_revision_mapping_sha256' if active_mapping else 'published_revision_mapping_sha256']=digest(source)
   else:
    old=None;obsolete=None;slug=re.sub(r'[^a-z0-9_-]+','_',model)
    if slug!=model:slug+='_'+hashlib.sha256(model.encode()).hexdigest()[:8]
    mp=cp.parent/'models'/slug/'module.json'
    if mp.exists():raise ValueError('New module path collision')
    definition={'schema':'unilab.robot-module/1','brand':brand,'model':model,'title':adapter['title'],
      'asset':binding,'provider':None,'asset_role':role,'composition':{'root_asset':binding},
      'capabilities':{'planning':'asset_adapter_available_not_engine_qualified','hardware':'not_qualified'},
      'motion_model':{'parameters':adapter.get('parameters',{}),'entrypoints':adapter['entrypoints']},
      'planning':{'adapter':'unilab_robot_runtime.asset_moveit','qualification':'not_run'},'drivers':{'status':'not_implemented'},
      'behavior':adapter.get('behavior_binding',{}),'source_provenance':{'sources':adapter.get('sources',[]),'published_revision_mapping_sha256':digest(source)}}
   pending.append((mp,definition,{'schema':'unilab.robot-module-assets/1','asset':binding}))
   cat['models'][model]={**(row or {}),'title':definition['title'],'asset':binding,'provider':definition['provider'],
     'capabilities':definition['capabilities'],'module_path':mp.relative_to(cp.parent).as_posix()}
   events.append({'brand':brand,'model':model,'status':'updated' if old else 'added','old':old,'new':binding,'invalidated_asset_adapter':obsolete,'drivers_preserved':bool(definition.get('drivers',{}).get('hardware_candidates'))})
 # No repository mutations occur until every target identity/digest/scope has
 # passed. Original module/lock backups above remain external evidence.
 for mp,definition,lock in pending:write(mp,definition);write(mp.parent/'assets.lock.json',lock)
 for cp,cat in catalogs.values():
  for entry in cat['models'].values():entry['module_sha256']=digest(cp.parent/entry['module_path'])
  write(cp,cat)
 result={'status':'passed','events':events,'models':sum(len(c['models']) for _,c in catalogs.values())};write(a.output/'receipt.json',result);print(json.dumps({'status':'passed','events':len(events),'models':result['models']}))

if __name__=='__main__':main()
