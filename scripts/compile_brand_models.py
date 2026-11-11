"""Batch exact-asset adapters into external cache; failures remain explicit."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import time
import os
import subprocess
import sys

from unilab_robot_contracts.brand_catalog import BrandCatalog
from unilab_robot_runtime.asset_moveit import compile_asset_model


def main():
    p=argparse.ArgumentParser();p.add_argument('--library',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--jobs',type=int,default=2);p.add_argument('--limit',type=int);p.add_argument('--worker',type=Path);a=p.parse_args()
    repo=Path(__file__).resolve().parents[1]
    if a.output.resolve().is_relative_to(repo):raise ValueError('Model caches must be outside repository')
    a.output.mkdir(parents=True,exist_ok=True);tasks=[]
    if a.worker:
        task=json.loads(a.worker.read_bytes());cat=BrandCatalog(task['catalog']);started=time.monotonic()
        row={'brand':task['brand'],'model':task['model'],'cache':task['cache']}
        try:
            d=cat.compile_asset_model(task['model'],a.library,a.output/task['cache'])
            row.update(status='compiled_not_engine_qualified',model_file_sha256=d['model_file_sha256'],asset=d['asset'],joints=len(d['joint_names']),triangles=sum(m['triangles'] for m in d['meshes']))
        except Exception as exc:row.update(status='rejected',reason=str(exc))
        row['seconds']=time.monotonic()-started
        a.worker.with_suffix('.result.json').write_text(json.dumps(row,ensure_ascii=False),encoding='utf-8')
        return
    for path in sorted((repo/'packages').glob('*/src/*/catalog.json')):
        cat=BrandCatalog(path.parent/'__init__.py')
        for name in cat.models():
            if cat.model(name).get('asset') and cat.module_definition(name).get('asset_role')!=['tool']:tasks.append((cat,name))
    if a.limit:tasks=tasks[:a.limit]
    progress=a.output/'progress.jsonl'
    rows=[json.loads(line) for line in progress.read_text(encoding='utf-8').splitlines()] if progress.exists() else []
    done={(r['brand'],r['model']) for r in rows}
    tasks=[t for t in tasks if (t[0]._read()['brand'],t[1]) not in done]
    def one(pair):
        cat,name=pair; brand=cat._read()['brand']; key=brand+'_'+hashlib.sha256(name.encode()).hexdigest()[:12]
        started=time.monotonic();row={'brand':brand,'model':name,'cache':key}
        if (a.output/key).exists():key+='-retry';row['cache']=key
        task_path=a.output/(key+'.task.json')
        task_path.write_text(json.dumps({**row,'catalog':str(cat.root/'__init__.py')}),encoding='utf-8')
        env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=os.pathsep.join(str(p) for p in (repo/'packages').glob('*/src')))
        try:
            with task_path.with_suffix('.log').open('w',encoding='utf-8') as log:
                completed=subprocess.run([sys.executable,'-B','-X','utf8',str(Path(__file__).resolve()),'--library',str(a.library),'--output',str(a.output),'--worker',str(task_path)],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=120)
            if completed.returncode:raise RuntimeError('Isolated USD compiler exited '+str(completed.returncode)+'; see '+task_path.with_suffix('.log').name)
            row=json.loads(task_path.with_suffix('.result.json').read_bytes())
        except Exception as exc: row.update(status='rejected',reason=str(exc))
        row['seconds']=time.monotonic()-started
        return row
    with ThreadPoolExecutor(max_workers=max(1,min(a.jobs,4))) as executor:
        for future in as_completed([executor.submit(one,t) for t in tasks]):
            rows.append(future.result())
            with (a.output/'progress.jsonl').open('a',encoding='utf-8') as stream:stream.write(json.dumps(rows[-1],ensure_ascii=False)+'\n')
            if len(rows)%100==0: print(json.dumps({'processed':len(rows),'total':len(done)+len(tasks)}),flush=True)
    (a.output/'receipt.json').write_text(json.dumps({'scope':'kinematic compiler only; not real engine qualification','results':rows},ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'processed':len(rows),'compiled':sum(x['status']=='compiled_not_engine_qualified' for x in rows)}))


if __name__=='__main__':main()
