"""Build the thin ERI C ABI bridge against an explicitly installed Linux SDK.

Run on the target Linux machine. SDK libraries remain external; this script
does not clone source repositories, connect to a controller, or grant a driver
qualification. The output must be outside this repository.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-root', type=Path, required=True)
    parser.add_argument('--expected-sdk-sha256', required=True)
    parser.add_argument('--expected-log4cpp-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compiler', default='g++')
    args = parser.parse_args()
    if platform.system() != 'Linux':
        raise RuntimeError('This SDK bridge requires Linux; no Windows DLL is supplied')
    repo = Path(__file__).resolve().parents[1]
    source = repo / 'packages/unilab-robot-runtime/src/unilab_robot_runtime/native/estun_eri_bridge.cpp'
    sdk = args.sdk_root.resolve(strict=True)
    output = args.output.resolve()
    if output.is_relative_to(repo) or output.is_relative_to(sdk):
        raise ValueError('Build output must be outside the repository and installed SDK')
    receipt = output.with_suffix('.build.json')
    log = output.with_suffix('.build.log')
    if any(p.exists() for p in (output, receipt, log)) or output.suffix != '.so':
        raise ValueError('Use a new external .so output path')
    paths = [sdk / p for p in (
        'include/ERIParamManager.h', 'include/EstunRobotERI.h',
        'lib/libEstunRobotERI.so', 'lib/liblog4cpp.so', 'config/log4cpp.conf',
        'LICENSE', 'REDISTRIBUTION.md', 'THIRD_PARTY_NOTICES.md',
        'LICENSES/ESTUN-SDK-REDISTRIBUTION.txt', 'LICENSES/LGPL-2.1.txt')]
    for path in paths:
        if not path.is_file() or not path.resolve().is_relative_to(sdk):
            raise ValueError(f'SDK dependency or license missing/outside installation: {path}')
    if sha256(sdk / 'lib/libEstunRobotERI.so') != args.expected_sdk_sha256:
        raise ValueError('ESTUN SDK digest differs from the selected version')
    if sha256(sdk / 'lib/liblog4cpp.so') != args.expected_log4cpp_sha256:
        raise ValueError('log4cpp digest differs from the selected version')
    # Absolute runtime path is explicit and recorded. No /work or source-repo
    # fallback is embedded; relocating the installation requires rebuilding.
    libdir = str(sdk / 'lib')
    command = [args.compiler, '-Wall', '-Wextra', '-Werror', '-std=c++17',
               '-shared', '-fPIC', str(source), '-I'+str(sdk/'include'),
               '-L'+libdir, '-lEstunRobotERI', '-llog4cpp',
               '-Wl,--disable-new-dtags', '-Xlinker', '-rpath', '-Xlinker',
               libdir, '-o', str(output)]
    output.parent.mkdir(parents=True, exist_ok=True)
    result = subprocess.run(command, capture_output=True, timeout=120)
    log.write_bytes(result.stdout + result.stderr)
    record = {'status': 'compiled' if result.returncode == 0 else 'compile_failed',
              'command': command, 'returncode': result.returncode,
              'source': {'path': str(source), 'sha256': sha256(source)},
              'sdk_root': str(sdk),
              'dependencies': [{'path': str(p.relative_to(sdk)), 'sha256': sha256(p)} for p in paths],
              'output': str(output), 'output_sha256': sha256(output) if output.is_file() else None,
              'qualification': 'build_only', 'hardware_connected': False}
    receipt.write_text(json.dumps(record, indent=2), encoding='utf-8')
    if result.returncode:
        raise RuntimeError(f'Bridge compilation failed; see {log}')
    print(json.dumps({'status': record['status'], 'receipt': str(receipt)}))


if __name__ == '__main__':
    main()
