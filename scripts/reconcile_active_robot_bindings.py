"""Audit every brand pin against the explicit local active index and installed wheels.

This maintenance command never silently follows latest during normal runtime.
It creates an exact, index-pinned reconciliation mapping for the existing
update_published_robot_bindings command. Physics/driver qualifications are not
granted by selecting the active asset revision.
"""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
FIELDS=('asset_id','package_digest','package_path')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,obj):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def installed():
    rows=[]
    for dist in importlib.metadata.distributions():
        for file in dist.files or []:
            if str(file).endswith('/catalog.json'):
                path=Path(dist.locate_file(file));catalog=json.loads(path.read_bytes())
                if catalog.get('schema')!='unilab.robot-brand/1':continue
                for model,entry in catalog['models'].items():
                    module=path.parent/entry['module_path'];definition=json.loads(module.read_bytes())
                    if sha(module)!=entry['module_sha256'] or definition['asset']!=entry['asset']:
                        raise ValueError('Installed catalog/module inconsistent')
                    rows.append({'brand':catalog['brand'],'model':model,'asset':entry['asset'],'module_sha256':sha(module),'drivers':definition.get('drivers',{}),'distribution':dist.metadata['Name']})
    print(json.dumps(rows,ensure_ascii=True))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--installed-inventory',action='store_true')
    p.add_argument('--library',type=Path);p.add_argument('--output',type=Path)
    p.add_argument('--installed-python',type=Path);p.add_argument('--require-current',action='store_true')
    a=p.parse_args()
    if a.installed_inventory:installed();return
    if not a.library or not a.output or not a.installed_python:p.error('library/output/installed-python required')
    if a.output.resolve().is_relative_to(ROOT):raise ValueError('Audit evidence must be outside repository')
    from unilab_robot_contracts._asset_package import verify,inside
    index_path=a.library/'library.json';index_sha=sha(index_path);index=json.loads(index_path.read_bytes())
    active={}
    for record in index['records']:
        for target in record.get('targets',[]):
            if target.get('member'):continue
            # Collection/alias records may point at catalog identities rather
            # than immutable packages; they cannot select a runtime pin.
            if not all(k in target for k in FIELDS):continue
            binding={k:target[k] for k in FIELDS}
            active.setdefault(target['asset_id'],{})[tuple(binding[k] for k in FIELDS)]=(binding,record)
    result=subprocess.run([str(a.installed_python),'-I','-B',str(Path(__file__).resolve()),'--installed-inventory'],capture_output=True,text=True,encoding='utf-8',check=True)
    installed_rows=json.loads(result.stdout);installed_map={(r['brand'],r['model']):r for r in installed_rows}
    if len(installed_map)!=len(installed_rows):raise ValueError('Duplicate installed brand/model')
    rows=[];targets=[];errors=[];before=[];verified={}
    for cp in sorted((ROOT/'packages').glob('*/src/*/catalog.json')):
        catalog=json.loads(cp.read_bytes())
        for model,item in catalog['models'].items():
            key=(catalog['brand'],model);module=cp.parent/item['module_path'];definition=json.loads(module.read_bytes())
            if sha(module)!=item['module_sha256'] or definition['asset']!=item['asset']:raise ValueError('Repository module/catalog inconsistent: '+str(key))
            before.append({'brand':key[0],'model':model,'asset':item['asset'],'drivers':definition.get('drivers',{})})
            candidates=active.get(item['asset']['asset_id'],{})
            if len(candidates)!=1:errors.append({'key':key,'reason':'active_asset_missing_or_ambiguous','count':len(candidates)});continue
            binding,record=next(iter(candidates.values()))
            if record.get('state') in ('failed','rejected','quarantined'):errors.append({'key':key,'reason':'active_record_rejected'});continue
            token=(binding['asset_id'],binding['package_digest'])
            if token not in verified:
                checked=verify(inside(a.library,binding['package_path']),binding['package_digest'])
                if checked['asset_id']!=binding['asset_id']:raise ValueError('Active identity mismatch')
                verified[token]={'asset_id':binding['asset_id'],'package_digest':binding['package_digest'],'package_path':binding['package_path'],'full_sha_verification':'passed'}
            installed_row=installed_map.get(key)
            row={'brand':key[0],'model':model,'roles':definition.get('asset_role'),'repository_pin':item['asset'],'installed_pin':installed_row['asset'] if installed_row else None,
                 'active_pin':binding,'repository_equals_active':item['asset']==binding,'installed_equals_repository':bool(installed_row and installed_row['asset']==item['asset'] and installed_row['module_sha256']==item['module_sha256']),
                 'active_state':record.get('state'),'active_qualification':record.get('qualification',{})}
            rows.append(row)
            if not row['repository_equals_active']:targets.append({'key':list(key),**binding})
    if set(installed_map)!={(r['brand'],r['model']) for r in before}:errors.append({'reason':'installed_repository_model_sets_differ'})
    if sha(index_path)!=index_sha:raise ValueError('Active index changed during audit; retry against a stable snapshot')
    summary={'status':'passed' if not errors else 'blocked','models':len(before),'active_index_sha256':index_sha,'active_packages_full_sha_verified':len(verified),
             'repository_active_differences':len(targets),'installed_repository_differences':sum(not r['installed_equals_repository'] for r in rows),'errors':errors}
    write(a.output/'audit.json',{'summary':summary,'rows':rows})
    write(a.output/'before-driver-snapshot.json',before)
    write(a.output/'current-scope.json',{'source':'Existing classified module asset_role; same asset identities, no new models',
        'rows':[{'brand':r['brand'],'model':r['model'],'roles':r['roles']} for r in rows]})
    write(a.output/'package-verification.json',list(verified.values()))
    write(a.output/'targets.json',{'status':'verified_local_active_revision_mapping','active_index_sha256':index_sha,'audit_sha256':sha(a.output/'audit.json'),'qualification_scope':'Local active revision, byte integrity verified; existing qualification limitations retained; not an OSS publication claim','targets':targets})
    print(json.dumps(summary))
    if errors or (a.require_current and (targets or summary['installed_repository_differences'])):raise SystemExit(1)

if __name__=='__main__':main()
