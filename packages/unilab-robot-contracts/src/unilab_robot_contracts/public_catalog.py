"""Resolve explicit public catalog bindings without changing any engineering pin.

The public module is a definition factory, not a hardware driver or OS Device
constructor. Loading a definition never connects, moves, or grants qualification.
"""
from __future__ import annotations

import hashlib
import importlib
import json
import re
from functools import lru_cache
from pathlib import Path
from uuid import UUID

from ._asset_package import inside
from .brand_catalog import BrandCatalog

PUBLIC_FIELDS = frozenset('uuid name display_name resource_type tags manufacturer_id description cover device_params model module source_registry relations'.split())
PIN_FIELDS = ('asset_id', 'package_digest', 'package_path')


@lru_cache(maxsize=1)
def _distributions():
    from importlib.metadata import packages_distributions
    return packages_distributions()


def read_entry(library_root: str | Path, uuid: str) -> dict:
    if str(UUID(uuid)) != uuid:
        raise ValueError('Expected canonical business UUID')
    root = Path(library_root) / '统一目录'
    index = json.loads((root / 'library.json').read_bytes())
    ref = index['files'][uuid]
    content = inside(root, ref['path']).read_bytes()
    if hashlib.sha256(content).hexdigest() != ref['sha256']:
        raise ValueError('Public entry changed; reindex the catalog')
    data = json.loads(content)
    if set(data) != PUBLIC_FIELDS or data['uuid'] != uuid:
        raise ValueError('Unsupported public entry contract or identity')
    return data


def validate_binding(catalog: BrandCatalog, model: str, entry: dict) -> dict:
    definition = catalog.module_definition(model)
    if definition.get('public_catalog') != {'contract': 'unilab.public-entry/2', 'uuid': entry['uuid']}:
        raise ValueError('Brand/public business identity mismatch')
    expected_module = catalog.root.name + ':module_definition'
    if entry['module'] != expected_module:
        raise ValueError('Public module does not match brand factory')
    selector = entry['model'].get('robot_module', {})
    if selector != {'distribution': catalog.root.parent.parent.name, 'model': model}:
        # Installed site-packages has no distribution directory; use import metadata.
        distributions = _distributions().get(catalog.root.name, [])
        if selector.get('model') != model or selector.get('distribution') not in distributions:
            raise ValueError('Public model selector differs from brand')
    matches = [t for t in entry['model'].get('targets', [])
               if t.get('asset_id') == definition['asset']['asset_id'] and not t.get('member')]
    if len(matches) != 1 or any(matches[0].get(k) != definition['asset'][k] for k in PIN_FIELDS):
        raise ValueError('Public/brand fixed asset binding differs; explicit migration required')
    return definition


def resolve_public_robot(uuid: str, library_root: str | Path) -> tuple[BrandCatalog, str]:
    """Resolve public UUID to installed brand/model; verify the complete asset package."""
    entry = read_entry(library_root, uuid)
    module = entry['module']
    if not isinstance(module, str) or not re.fullmatch(r'unilab_robot_[a-z0-9_]+:module_definition', module):
        raise ValueError('Not a registered robot definition factory')
    brand = importlib.import_module(module.split(':')[0])
    catalog = BrandCatalog(brand.__file__)
    model = entry['model']['robot_module']['model']
    validate_binding(catalog, model, entry)
    catalog.asset(model, library_root)
    return catalog, model
