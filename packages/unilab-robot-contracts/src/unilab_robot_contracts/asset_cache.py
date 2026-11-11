"""Download an exact NVIDIA asset package into an external cache.

Transport supplies bytes by relative object key; no credentials or store URL are
embedded in brand packages. Incomplete downloads never become active assets.
"""
import json
from pathlib import Path
import tempfile
import os

from ._asset_package import MANIFEST, BOM, inside, sha, verify


def resolve_asset_files(lock_path, library_root=None):
    """Resolve frozen legacy geometry by hash from the same standard library."""
    lock=json.loads(Path(lock_path).read_bytes())
    library_root=library_root or os.environ.get('UNILAB_ASSET_LIBRARY')
    if not library_root:
        raise ValueError('Set UNILAB_ASSET_LIBRARY or pass library_root; brand packages do not contain geometry')
    root=inside(Path(library_root),lock['asset']['package_path'])
    result=verify(root,lock['asset']['package_digest'])
    if result['asset_id']!=lock['asset']['asset_id']:raise ValueError('Legacy geometry asset identity mismatch')
    files={}
    for item in lock['files']:
        path=inside(root,item['asset_path'])
        if sha(path.read_bytes())!=item['sha256']:raise ValueError('Legacy geometry hash mismatch')
        files[item['original']]=path
    return files


def fetch_asset(binding, cache_root, read_object):
    root = Path(cache_root).resolve()
    destination = inside(root, binding['package_path'])
    if destination.exists():
        result = verify(destination, binding['package_digest'])
        if result['asset_id'] != binding['asset_id']:
            raise ValueError('Cached asset identity mismatch')
        return destination
    root.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix='.asset-download-', dir=root))
    package = staging / destination.name
    package.mkdir()
    prefix = binding['package_path'] + '/'

    def get(rel, expected, size=None):
        path = inside(package, rel)
        data = read_object(prefix + rel)
        if sha(data) != expected or size is not None and len(data) != size:
            raise ValueError('Downloaded asset hash/size mismatch: ' + rel)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        return data

    try:
        manifest = json.loads(get(MANIFEST, binding['package_digest']))
        if manifest['package_id'] != binding['asset_id']:
            raise ValueError('Remote asset identity mismatch')
        for item in manifest['metadata']:
            get('.metadata/' + item['name'], item['hash']['sha256'])
        bom = json.loads((package / BOM).read_bytes())
        seen = {MANIFEST.casefold(), *('.metadata/' + v['name'].casefold() for v in manifest['metadata'])}
        for item in bom['items']:
            path = item['relative_path']
            if path.casefold() in seen:
                raise ValueError('Duplicate package path')
            seen.add(path.casefold())
            get(path, item['hash']['sha256'], item['size'])
        verify(package, binding['package_digest'])
        destination.parent.mkdir(parents=True, exist_ok=True)
        package.rename(destination)
        staging.rmdir()
        return destination
    except Exception as exc:
        (staging / 'failure.json').write_text(json.dumps({'asset':binding, 'error':str(exc)}), encoding='utf-8')
        raise
