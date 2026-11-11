"""Verify installed public/brand/asset associations and representative compilations."""
import argparse
import copy
import hashlib
import importlib
import json
from pathlib import Path
from unilab_robot_contracts.brand_catalog import BrandCatalog
from unilab_robot_contracts.public_catalog import validate_binding, resolve_public_robot, PUBLIC_FIELDS

REPRESENTATIVES = {'dobot': 'cr5', 'elite': 'ec66', 'abb': 'irb1200:5:0.9',
                   'fanuc': 'lrmate200ic', 'kuka': 'kr6r900sixx'}


def main():
    p = argparse.ArgumentParser(); p.add_argument('--library', type=Path, required=True)
    p.add_argument('--mapping', type=Path, required=True); p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=False)
    index = json.loads((a.library/'统一目录/library.json').read_bytes())
    mapping = json.loads(a.mapping.read_bytes())['rows']; rows = []; compiled = []; negatives = []
    brands = {}
    for item in mapping:
        name = item['module'].split(':')[0]
        if name not in brands: brands[name] = BrandCatalog(importlib.import_module(name).__file__)
        cat = brands[name]; uuid = item['uuid']; ref = index['files'][uuid]
        raw = (a.library/'统一目录'/ref['path']).read_bytes()
        assert hashlib.sha256(raw).hexdigest() == ref['sha256']
        entry = json.loads(raw); assert set(entry) == PUBLIC_FIELDS
        definition = validate_binding(cat, item['model'], entry)
        cat.asset(item['model'], a.library)
        rows.append({**item, 'status': 'passed', 'module_sha256': cat.model(item['model'])['module_sha256']})
        if REPRESENTATIVES.get(item['brand']) == item['model']:
            resolved, model = resolve_public_robot(uuid, a.library)
            output = a.output/'models'/item['brand']
            result = resolved.compile_asset_model(model, a.library, output)
            compiled.append({**item, 'output': str(output), 'model_file_sha256': result['model_file_sha256']})
            for field, change in [('package_digest', '0'*64), ('package_path', '../outside'), ('asset_id', 'wrong')]:
                bad = copy.deepcopy(entry); bad['model']['targets'][0][field] = change
                try: validate_binding(cat, model, bad)
                except ValueError: negatives.append({'model': model, 'case': field, 'rejected': True})
                else: raise AssertionError('Accepted invalid ' + field)
            bad = copy.deepcopy(entry); bad['uuid'] = '00000000-0000-0000-0000-000000000000'
            try: validate_binding(cat, model, bad)
            except ValueError: negatives.append({'model': model, 'case': 'business_uuid', 'rejected': True})
            else: raise AssertionError('Accepted wrong business identity')
    assert len(compiled) == 5
    report = {'passed': True, 'models': len(rows), 'distributions': len(brands),
              'rows': rows, 'compiled': compiled, 'negative_tests': negatives,
              'scope': 'installed bindings, full package checks and compilation; engine validation is separate'}
    (a.output/'receipt.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k in ('passed','models','distributions')}))


if __name__ == '__main__': main()
