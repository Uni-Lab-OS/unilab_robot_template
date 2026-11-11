"""Incrementally bind the three-source union; preserve every existing module/lock.

Input crosswalk comes from the maintenance comparison, never runtime scans.
Unpublished, ambiguous, static and template-only candidates remain in the receipt.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re

from import_brand_catalogs import ROOT, BRAND_INIT, brand_key, write, digest


def extend(library, crosswalk, receipt, scope_report=None):
    from pxr import Usd, UsdPhysics
    from unilab_robot_contracts._asset_package import verify
    index=json.loads((library/'library.json').read_bytes())
    targets={t['asset_id']:t for r in index['records'] for t in r.get('targets',[])}
    brands={}; owned={}; outcomes=[]
    for path in (ROOT/'packages').glob('*/src/*/catalog.json'):
        d=json.loads(path.read_bytes()); brands[d['brand']]=(path,d)
        for name,row in d['models'].items():
            if row.get('asset'): owned[row['asset']['asset_id']]=(d['brand'],name)
    candidates=json.loads(crosswalk.read_bytes())
    groups=candidates.get('groups',candidates.get('matches',[]))
    if not groups: raise ValueError('Empty or unknown source crosswalk')
    if scope_report is None:raise ValueError('A classified arms/tools --scope-report is required')
    scope=json.loads(scope_report.read_bytes())
    allowed={(brand_key(r['brand']),r['model']):r for r in scope['rows']}
    added=[]
    for g in groups:
        name=g['model_key']; sources=g['sources']
        try: b=brand_key(g['brand_key'])
        except ValueError:
            outcomes.append({'brand':g['brand_key'],'model':name,'status':'manufacturer_identity_needs_review'});continue
        role=allowed.get((b,name))
        if role is None:continue
        row={'brand':b,'model':name,'sources':sorted(sources)}
        if b in brands and name in brands[b][1]['models']:
            row['status']='existing_module_preserved'; outcomes.append(row); continue
        ids={i for i in role['formal_asset_ids'] if i in targets}
        if any(i in owned for i in ids):
            row.update(status='existing_asset_identity_candidate',existing=[owned[i] for i in ids if i in owned]);outcomes.append(row);continue
        if len(ids)!=1:
            row.update(status='not_published' if not ids else 'ambiguous_multiple_assets',asset_ids=sorted(ids));outcomes.append(row);continue
        target=targets[next(iter(ids))]; package=library/target['package_path']
        try:
            verify(package,target['package_digest'])
            adapter=json.loads((package/'simready_usd/unilab.json').read_bytes())
            stage=Usd.Stage.Open(str(package/adapter['entrypoints']['usd']))
            count=sum(p.IsA(UsdPhysics.Joint) and not p.IsA(UsdPhysics.FixedJoint) for p in stage.Traverse())
            expected_moving=any((r.get('axes') or 0)>0 for rows in sources.values() for r in rows) or bool(adapter.get('parameters',{}).get('kinematics',{}).get('joints'))
            if count==0 and ('arm' in role['roles'] or expected_moving): raise ValueError('No articulated USD joints; source mechanism cannot become static')
            binding={k:target[k] for k in ('asset_id','package_digest','package_path')}
            if b not in brands:
                module='unilab_robot_'+b; dist='unilab-robot-'+b.replace('_','-'); package_root=ROOT/'packages'/dist
                path=package_root/'src'/module/'catalog.json';path.parent.mkdir(parents=True,exist_ok=True)
                (path.parent/'__init__.py').write_text(BRAND_INIT,encoding='utf-8')
                (package_root/'pyproject.toml').write_text('[build-system]\nrequires = ["setuptools>=68", "wheel"]\nbuild-backend = "setuptools.build_meta"\n[project]\nname = "'+dist+'"\nversion = "0.1.0"\nrequires-python = ">=3.11"\ndependencies = ["unilab-robot-contracts>=0.1,<0.2"]\n[tool.setuptools.packages.find]\nwhere = ["src"]\n[tool.setuptools.package-data]\n'+module+' = ["catalog.json", "models/**/*", "source_profiles/**/*"]\n',encoding='utf-8')
                brands[b]=(path,{'schema':'unilab.robot-brand/1','brand':b,'models':{},'source_profiles':[]})
            path, catalog=brands[b];slug=re.sub(r'[^a-z0-9_-]+','_',name)
            if slug!=name: slug+='_'+hashlib.sha256(name.encode()).hexdigest()[:8]
            folder=path.parent/'models'/slug
            caps={'planning':'asset_adapter_available_not_engine_qualified','hardware':'not_qualified'}
            if role['roles']==['tool']:caps['planning']='tool_geometry_binding_not_arm_planner'
            definition={'schema':'unilab.robot-module/1','brand':b,'model':name,'title':adapter['title'],'asset':binding,'provider':None,'capabilities':caps,
                        'composition':{'root_asset':binding},'motion_model':{'parameters':adapter.get('parameters',{}),'entrypoints':adapter['entrypoints']},
                        'planning':{'adapter':'unilab_robot_runtime.asset_moveit' if 'arm' in role['roles'] else None,'usd_joints':count,'source_profile_candidates':[]},
                        'asset_role':role['roles'],
                        'drivers':{'status':'not_implemented'},'behavior':adapter.get('behavior_binding',{}),
                        'source_provenance':{'crosswalk_sources':sources,'sources':adapter.get('sources',[])}}
            write(folder/'module.json',definition);write(folder/'assets.lock.json',{'schema':'unilab.robot-module-assets/1','asset':binding})
            catalog['models'][name]={'title':adapter['title'],'asset':binding,'provider':None,'capabilities':caps,'module_path':(folder/'module.json').relative_to(path.parent).as_posix(),'module_sha256':digest(folder/'module.json')}
            owned[binding['asset_id']]=(b,name);row.update(status='added',asset=binding,usd_joints=count);added.append(row)
        except Exception as exc: row.update(status='rejected',reason=str(exc))
        outcomes.append(row)
    for path,catalog in brands.values(): write(path,catalog)
    summary={'schema':'unilab.robot-union-import/1','crosswalk_sha256':digest(crosswalk),'library_index_sha256':digest(library/'library.json'),
             'added':len(added),'brands':len(brands),'models':sum(len(c['models']) for _,c in brands.values()),'outcomes':outcomes}
    write(receipt,summary);print(json.dumps({k:v for k,v in summary.items() if k!='outcomes'}))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--library',type=Path,required=True);p.add_argument('--crosswalk',type=Path,required=True);p.add_argument('--receipt',type=Path,required=True);p.add_argument('--scope-report',type=Path,required=True);a=p.parse_args()
    extend(a.library,a.crosswalk,a.receipt,a.scope_report)
