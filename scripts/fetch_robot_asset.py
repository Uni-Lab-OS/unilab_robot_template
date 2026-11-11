"""Fetch a pinned brand asset from OSS (credentials only in environment)."""
import argparse
import importlib
import json
from pathlib import Path

from unilab_robot_contracts.asset_cache import fetch_asset


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--brand',required=True); p.add_argument('--model',required=True)
    p.add_argument('--cache',required=True,type=Path); p.add_argument('--store',required=True,type=Path)
    a=p.parse_args()
    import oss2
    from oss2.credentials import EnvironmentVariableCredentialsProvider
    config=json.loads(a.store.read_bytes())
    bucket=oss2.Bucket(oss2.ProviderAuthV4(EnvironmentVariableCredentialsProvider()),config['endpoint'],config['bucket'],region=config['region'])
    catalog=importlib.import_module('unilab_robot_'+a.brand).catalog
    binding=catalog.module_definition(a.model)['asset']
    path=fetch_asset(binding,a.cache,lambda key:bucket.get_object(config['prefix']+key).read())
    print(json.dumps({'model':a.model,'asset':binding,'local_path':str(path)},ensure_ascii=False))


if __name__=='__main__': main()
