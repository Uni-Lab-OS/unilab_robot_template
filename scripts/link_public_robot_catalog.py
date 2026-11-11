"""Link brand definitions to v2 public entries by exact pins, never by names.

Preflight the entire batch before writes. --apply keeps original files outside
the repositories; rebuild the public index afterwards with unified.py reindex.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'packages/unilab-robot-contracts/src'))
from unilab_robot_contracts.public_catalog import PUBLIC_FIELDS, PIN_FIELDS


def encoded(value):
    return (json.dumps(value, ensure_ascii=False, indent=2) + '\n').encode('utf-8')


def plan(library: Path):
    by_asset = defaultdict(list)
    for path in sorted((library / '统一目录/条目').glob('*.json')):
        content = path.read_bytes(); entry = json.loads(content)
        if set(entry) != PUBLIC_FIELDS or path.stem != entry['uuid']:
            raise ValueError(f'Invalid public entry: {path}')
        model = entry.get('model')
        for target in (model.get('targets', []) if isinstance(model, dict) else []):
            if not target.get('member'):
                by_asset[target.get('asset_id')].append((path, content, entry, target))
    changes = {}; rows = []; seen = set()
    for path in sorted((REPO / 'packages').glob('unilab-robot-*/src/*/catalog.json')):
        original = path.read_bytes(); catalog = json.loads(original)
        for model, row in catalog['models'].items():
            pin = row['asset']; candidates = by_asset[pin['asset_id']]
            if len(candidates) != 1:
                raise ValueError(f'{path.parent.name}/{model}: missing or ambiguous public target')
            public_path, public_bytes, entry, target = candidates[0]
            if entry['uuid'] in seen:
                raise ValueError('Multiple modules own one public entry: ' + entry['uuid'])
            seen.add(entry['uuid'])
            if any(pin[k] != target.get(k) for k in PIN_FIELDS):
                raise ValueError('Fixed pin mismatch: ' + entry['uuid'])
            module_path = path.parent / row['module_path']
            if not module_path.resolve().is_relative_to(path.parent.resolve()):
                raise ValueError('Module path outside brand')
            module_bytes = module_path.read_bytes(); definition = json.loads(module_bytes)
            if hashlib.sha256(module_bytes).hexdigest() != row['module_sha256'] or definition['asset'] != pin:
                raise ValueError('Brand definition integrity failure: ' + str(module_path))
            if definition['asset_role'] not in (['arm'], ['tool']):
                raise ValueError('Outside robot/tool scope')
            factory = path.parent.name + ':module_definition'
            selector = {'distribution': path.parents[2].name, 'model': model}
            reverse = {'contract': 'unilab.public-entry/2', 'uuid': entry['uuid']}
            if entry['module'] not in (None, '', factory):
                raise ValueError('Existing public module conflict: ' + entry['uuid'])
            if entry['model'].get('robot_module', selector) != selector or definition.get('public_catalog', reverse) != reverse:
                raise ValueError('Existing robot association conflict: ' + entry['uuid'])
            entry['module'] = factory; entry['model']['robot_module'] = selector
            definition['public_catalog'] = reverse
            new_module = encoded(definition)
            row['module_sha256'] = hashlib.sha256(new_module).hexdigest()
            for p, old, new in [(public_path, public_bytes, encoded(entry)), (module_path, module_bytes, new_module)]:
                if old != new: changes[p] = (old, new)
            rows.append({'uuid': entry['uuid'], 'brand': catalog['brand'], 'model': model,
                         'module': factory, 'asset': pin, 'role': definition['asset_role'][0]})
        new = encoded(catalog)
        if original != new: changes[path] = (original, new)
    return changes, rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--apply', action='store_true')
    a = p.parse_args()
    if a.output.resolve().is_relative_to(REPO) or a.output.resolve().is_relative_to(a.library.resolve()):
        p.error('Evidence must be outside repository and library')
    a.output.mkdir(parents=True, exist_ok=False)
    changes, rows = plan(a.library)
    receipt = {'applied': False, 'models': len(rows), 'brands': len({r['brand'] for r in rows}),
               'roles': dict(Counter(r['role'] for r in rows)), 'changed_files': len(changes), 'rows': rows}
    (a.output / 'receipt.json').write_bytes(encoded(receipt))
    if a.apply:
        # Reject concurrent edits before applying anything, preserve every original.
        for path, (old, _) in changes.items():
            if path.read_bytes() != old: raise ValueError('Concurrent edit: ' + str(path))
        backup = a.output / 'before'; backup.mkdir()
        manifest = []
        for i, (path, (old, new)) in enumerate(changes.items()):
            (backup / str(i)).write_bytes(old)
            manifest.append({'path': str(path), 'backup': str(i), 'before': hashlib.sha256(old).hexdigest(), 'after': hashlib.sha256(new).hexdigest()})
        (a.output / 'changes.json').write_bytes(encoded(manifest))
        for path, (old, new) in changes.items():
            if path.read_bytes() != old: raise ValueError('Concurrent edit: ' + str(path))
            temp = path.with_name(path.name + '.robot-binding.tmp'); temp.write_bytes(new); temp.replace(path)
        receipt['applied'] = True
        (a.output / 'receipt.json').write_bytes(encoded(receipt))
    print(json.dumps({k:v for k,v in receipt.items() if k != 'rows'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
