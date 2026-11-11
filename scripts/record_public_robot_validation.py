"""Record real engine/OS receipts for exact pins; never grant hardware support."""
import argparse
import hashlib
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def encode(value):
    return (json.dumps(value, ensure_ascii=False, indent=2)+'\n').encode('utf-8')


def main():
    p = argparse.ArgumentParser(); p.add_argument('--evidence', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True); a = p.parse_args()
    if a.output.resolve().is_relative_to(REPO): raise ValueError('Use external evidence output')
    rows = json.loads((a.evidence/'selected.json').read_bytes()); changes = {}; accepted = []; catalogs = {}
    exits = {}
    for filename in ('process-exits.txt', 'os-process-exits.txt'):
        for line in (a.evidence/'engine'/filename).read_text().splitlines():
            brand, kind, code = line.split(); exits[(brand,kind)] = int(code)
    for row in rows:
        assert exits[(row['brand'],'engine')] == exits[(row['brand'],'os')] == 0
        name = row['module'].split(':')[0]
        paths = list((REPO/'packages').glob('*/src/'+name+'/catalog.json'))
        if len(paths) != 1: raise ValueError('Missing/ambiguous brand')
        path = paths[0]
        if path not in catalogs:
            raw = path.read_bytes(); catalogs[path] = (raw, json.loads(raw))
        raw, cat = catalogs[path]
        item = cat['models'][row['model']]; module = path.parent/item['module_path']
        original = module.read_bytes(); definition = json.loads(original)
        assert hashlib.sha256(original).hexdigest() == item['module_sha256']
        assert item['asset'] == row['asset'] == definition['asset']
        engine_path = a.evidence/'engine'/f"{row['brand']}.json"
        os_path = a.evidence/'engine'/f"{row['brand']}-os.json"
        engine = json.loads(engine_path.read_bytes()); os = json.loads(os_path.read_bytes())
        assert engine['passed'] and os['passed'] and engine['asset'] == item['asset']
        assert os['catalog_uuid'] == row['uuid'] and os['forward']['asset'] == item['asset']
        assert os['forward']['hardware_connected'] is False
        status = 'validated_moveit_fcl_and_os_usd_kinematics'
        qualification = {'status': status, 'asset': item['asset'], 'catalog_uuid': row['uuid'],
            'compiled_model_sha256': hashlib.sha256((a.evidence/'verified-02/models'/row['brand']/'model.json').read_bytes()).hexdigest(),
            'evidence': {engine_path.name: hashlib.sha256(engine_path.read_bytes()).hexdigest(),
                         os_path.name: hashlib.sha256(os_path.read_bytes()).hexdigest()},
            'scope': 'joint/pose planning, sampled FCL, OS JSON driver dispatch, USD kinematic motion; no hardware, dynamics or full workflow',
            'simulation_driver': os['driver_class']}
        definition['planning']['public_catalog_validation'] = qualification
        definition['planning']['asset_adapter_status'] = status
        definition['capabilities']['asset_planning'] = status
        new = encode(definition); item['module_sha256'] = hashlib.sha256(new).hexdigest()
        changes[module] = (original, new); changes[path] = (raw, encode(cat)); accepted.append(qualification)
    a.output.mkdir(parents=True, exist_ok=False)
    manifest = []
    for i,(path,(old,new)) in enumerate(changes.items()):
        assert path.read_bytes() == old
        backup = str(i)+'.before'; (a.output/backup).write_bytes(old)
        manifest.append({'path':str(path),'backup':backup,'after_sha256':hashlib.sha256(new).hexdigest()})
    (a.output/'changes.json').write_bytes(encode(manifest))
    for path,(old,new) in changes.items():
        assert path.read_bytes() == old
        temp=path.with_name(path.name+'.validation.tmp');temp.write_bytes(new);temp.replace(path)
    (a.output/'receipt.json').write_bytes(encode({'accepted':accepted}))
    print('Recorded exact-pin qualifications:',len(accepted))


if __name__ == '__main__': main()
