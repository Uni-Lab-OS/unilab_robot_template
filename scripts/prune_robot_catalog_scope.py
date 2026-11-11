"""Keep only evidenced arms/tools; consolidate explicit manufacturer aliases.

Removed generated modules move to the supplied external review directory.
Published library bytes and existing consumers' asset locks are never modified.
"""
import argparse
import json
from pathlib import Path
import shutil

from import_brand_catalogs import ROOT, BRAND_INIT, brand_key, write, digest


def main():
    p=argparse.ArgumentParser();p.add_argument('--scope-report',type=Path,required=True);p.add_argument('--review',type=Path,required=True);a=p.parse_args()
    review=a.review.resolve();repo=ROOT.resolve()
    if review.is_relative_to(repo):raise ValueError('Review/archive must be outside repository')
    report=json.loads(a.scope_report.read_bytes());allowed={i for r in report['rows'] for i in r['formal_asset_ids']};events=[]
    def move(source,destination):
        source=source.resolve();destination=destination.resolve()
        if not source.is_relative_to(repo/'packages') or not destination.is_relative_to(review):raise ValueError('Unsafe archive path')
        if destination.exists():raise ValueError('Review destination already exists')
        destination.parent.mkdir(parents=True,exist_ok=True);shutil.move(str(source),str(destination))
    # Explicit established providers have reviewed robot URDFs, including the
    # independently maintained CR12 which is not a raw crosswalk identity.
    providers={'unilab_arm_cr5:build_moveit_model','unilab_arm_cr7:build_moveit_model','unilab_arm_rokae_cr12:build_moveit_model'}
    for path in sorted((repo/'packages').glob('*/src/*/catalog.json')):
        catalog=json.loads(path.read_bytes());changed=False
        for model,row in list(catalog['models'].items()):
            identity=(row.get('asset') or {}).get('asset_id')
            if identity in allowed or row.get('provider') in providers:continue
            folder=(path.parent/row['module_path']).parent
            move(folder,review/'out-of-scope'/catalog['brand']/folder.name)
            del catalog['models'][model];changed=True
            events.append({'action':'removed_from_brand_catalog','brand':catalog['brand'],'model':model,'asset':row.get('asset'),'reason':'No verified arm/end-effector role in scoped source inventory'})
        if changed:write(path,catalog)
    for path in sorted((repo/'packages').glob('*/src/*/catalog.json')):
        if not path.exists():continue
        old=json.loads(path.read_bytes());canonical=brand_key(old['brand'])
        if canonical==old['brand']:continue
        dist='unilab-robot-'+canonical.replace('_','-');module='unilab_robot_'+canonical
        target_root=repo/'packages'/dist;target=target_root/'src'/module;catalog_path=target/'catalog.json'
        if catalog_path.exists():new=json.loads(catalog_path.read_bytes())
        else:
            target.mkdir(parents=True,exist_ok=True);(target/'__init__.py').write_text(BRAND_INIT,encoding='utf-8')
            old_pyproject=path.parents[2]/'pyproject.toml'
            text=old_pyproject.read_text(encoding='utf-8').replace('unilab-robot-'+old['brand'].replace('_','-'),dist).replace('unilab_robot_'+old['brand'],module)
            (target_root/'pyproject.toml').write_text(text,encoding='utf-8')
            new={'schema':'unilab.robot-brand/1','brand':canonical,'models':{},'source_profiles':[]}
        if old.get('source_profiles'):raise ValueError('Alias has frozen source profiles; explicit migration needed')
        for model,row in old['models'].items():
            if model in new['models']:
                events.append({'action':'alias_binding_conflict_preserved_for_review','brand':canonical,'model':model,'kept':new['models'][model]['asset'],'candidate':row['asset']});continue
            source=path.parent/row['module_path'];dest=target/row['module_path']
            if dest.parent.exists():raise ValueError('Alias target module folder collision')
            dest.parent.parent.mkdir(parents=True,exist_ok=True);shutil.copytree(source.parent,dest.parent)
            definition=json.loads(dest.read_bytes());definition['brand']=canonical;definition.setdefault('source_provenance',{})['source_brand_alias']=old['brand'];write(dest,definition)
            row['module_sha256']=digest(dest);new['models'][model]=row
            events.append({'action':'manufacturer_alias_consolidated','old_brand':old['brand'],'brand':canonical,'model':model})
        write(catalog_path,new)
        move(path.parents[2],review/'brand-alias-packages'/path.parents[2].name)
    totals=[]
    for path in (repo/'packages').glob('*/src/*/catalog.json'):
        d=json.loads(path.read_bytes());totals.append({'brand':d['brand'],'models':len(d['models'])})
    write(review/'receipt.json',{'scope_report_sha256':digest(a.scope_report),'events':events,'brands':totals,'model_count':sum(r['models'] for r in totals)})
    print(json.dumps({'brands':len(totals),'models':sum(r['models'] for r in totals),'removed':sum(r['action']=='removed_from_brand_catalog' for r in events),'aliases':sum(r['action']=='manufacturer_alias_consolidated' for r in events),'binding_conflicts':sum(r['action']=='alias_binding_conflict_preserved_for_review' for r in events)}))


if __name__=='__main__':main()
