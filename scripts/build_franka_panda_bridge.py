"""Build a read-only Panda bridge against an explicitly installed libfranka 0.9.2.

Run on Linux. Install the pinned SDK separately; this command neither downloads
dependencies nor contacts a controller. Shared libraries remain external.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sdk-prefix', type=Path, required=True)
    p.add_argument('--expected-library-sha256', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--compiler', default='g++')
    a = p.parse_args()
    if platform.system() != 'Linux':
        raise RuntimeError('This bridge is validated only on Linux')
    repo = Path(__file__).resolve().parents[1]
    sdk = a.sdk_prefix.resolve(strict=True)
    library = (sdk / 'lib/libfranka.so.0.9.2').resolve(strict=True)
    if not library.is_relative_to(sdk) or digest(library) != a.expected_library_sha256:
        raise ValueError('Selected Panda SDK binary differs from expected build')
    output = a.output.resolve()
    if output.exists() or output.suffix != '.so' or output.is_relative_to(repo) or output.is_relative_to(sdk):
        raise ValueError('Use a fresh external .so output')
    bridge = repo / 'packages/unilab-robot-runtime/src/unilab_robot_runtime/native/franka_panda_bridge.cpp'
    output.parent.mkdir(parents=True, exist_ok=True)
    command = [a.compiler, '-std=c++14', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
               '-I'+str(sdk/'include'), str(bridge), str(library),
               '-Wl,-rpath,'+str(library.parent), '-o', str(output)]
    subprocess.run(command, check=True)
    receipt = {'sdk': 'libfranka', 'version': '0.9.2', 'model_generation': 'Panda',
               'library_sha256': digest(library), 'bridge_source_sha256': digest(bridge),
               'bridge_sha256': digest(output), 'compiler': shutil.which(a.compiler),
               'command': command, 'capabilities': ['connect', 'read_joints'],
               'motion_supported': False, 'hardware_verified': False}
    output.with_suffix('.receipt.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(receipt, indent=2))


if __name__ == '__main__':
    main()
