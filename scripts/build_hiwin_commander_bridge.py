"""Build pinned HIWIN client sources and a thin C ABI outside the repository.

The build copy adds checked TCP exchanges because upstream Commander ignores
read/write failures. The exact diff, upstream hashes and licenses accompany the
result. No controller is contacted by this maintenance command.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sdk-root',type=Path,required=True)
    p.add_argument('--expected-source-tree-sha256',required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--compiler',default='g++')
    a=p.parse_args()
    if platform.system()!='Linux' or sys.byteorder!='little':
        raise RuntimeError('Pinned HIWIN transport requires little-endian Linux')
    repo=Path(__file__).resolve().parents[1];sdk=a.sdk_root.resolve(strict=True)
    output=a.output.resolve();build=output.parent/(output.stem+'-build')
    if output.suffix!='.so' or output.is_relative_to(repo) or output.is_relative_to(sdk):
        raise ValueError('Use an external .so output outside repository and SDK')
    if output.exists() or build.exists():raise ValueError('Use a fresh output path')
    names=['src/commander.cpp','src/socket/tcp_client.cpp','LICENSE','LICENSE_bsd_3_clause','LICENSE_apache_2_0']
    names+=sorted(p.relative_to(sdk).as_posix() for p in (sdk/'include').rglob('*.hpp'))
    for name in names:
        path=sdk/name
        if not path.is_file() or not path.resolve().is_relative_to(sdk):raise ValueError('Missing SDK input '+name)
    entries=[{'path':n,'sha256':digest(sdk/n)} for n in sorted(names)]
    tree=hashlib.sha256(''.join(x['path']+'\0'+x['sha256']+'\n' for x in entries).encode()).hexdigest()
    if tree!=a.expected_source_tree_sha256:raise ValueError('SDK source tree differs from selected revision')
    original=(sdk/'src/commander.cpp').read_text(encoding='utf-8')
    write='TCPClient::write(data_w, sizeof(Commandformat), written);'
    read='TCPClient::read(data_r, sizeof(Responseformat), read_chars);'
    nw=original.count(write);nr=original.count(read)
    if not nw or nw!=nr:raise ValueError('Unreviewed SDK transport layout')
    checked_write='''if (!TCPClient::write(data_w, sizeof(Commandformat), written) || written != sizeof(Commandformat))
    throw std::runtime_error("HIWIN incomplete request");'''
    checked_read='''read_chars = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (read_chars < sizeof(Responseformat)) {
    size_t part = 0;
    if (std::chrono::steady_clock::now() >= deadline ||
        !TCPClient::read(data_r + read_chars, sizeof(Responseformat) - read_chars, part) || !part)
      throw std::runtime_error("HIWIN incomplete response or timeout");
    read_chars += part;
  }
  if (r.cmd_id != w.cmd_id) throw std::runtime_error("HIWIN mismatched response command");'''
    modified=original.replace('#include <cmath>','#include <cmath>\n#include <stdexcept>\n#include <chrono>')
    modified=modified.replace(write,checked_write).replace(read,checked_read)
    tcp=(sdk/'src/socket/tcp_client.cpp').read_text(encoding='utf-8')
    send='::send(socket_fd_, buf + written, remaining, 0)'
    if tcp.count(send)!=1:raise ValueError('Unreviewed SDK send layout')
    tcp_modified=tcp.replace(send,'::send(socket_fd_, buf + written, remaining, MSG_NOSIGNAL)')
    build.mkdir(parents=True)
    for name in names:
        target=build/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(sdk/name,target)
    (build/'src/commander.cpp').write_text(modified,encoding='utf-8')
    (build/'src/socket/tcp_client.cpp').write_text(tcp_modified,encoding='utf-8')
    diff=''.join(difflib.unified_diff(original.splitlines(True),modified.splitlines(True),fromfile='upstream/src/commander.cpp',tofile='checked/src/commander.cpp'))
    diff+=''.join(difflib.unified_diff(tcp.splitlines(True),tcp_modified.splitlines(True),fromfile='upstream/src/socket/tcp_client.cpp',tofile='checked/src/socket/tcp_client.cpp'))
    (build/'transport-checks.patch').write_text(diff,encoding='utf-8')
    bridge=repo/'packages/unilab-robot-runtime/src/unilab_robot_runtime/native/hiwin_commander_bridge.cpp'
    command=[a.compiler,'-std=c++17','-shared','-fPIC','-pthread',str(bridge),str(build/'src/commander.cpp'),str(build/'src/socket/tcp_client.cpp'),'-I'+str(build/'include'),'-o',str(output)]
    result=subprocess.run(command,capture_output=True,timeout=120)
    (build/'compile.log').write_bytes(result.stdout+result.stderr)
    receipt={'status':'compiled' if result.returncode==0 else 'failed','source_tree_sha256':tree,'inputs':entries,'source_sdk':str(sdk),'sdk_transport_modified':True,'patch_sha256':digest(build/'transport-checks.patch'),'checked_exchanges':nw,'bridge_sha256':digest(bridge),'command':command,'returncode':result.returncode,'output':str(output),'output_sha256':digest(output) if output.exists() else None,'qualification':'build_only','hardware_connected':False}
    (build/'receipt.json').write_text(json.dumps(receipt,indent=2),encoding='utf-8')
    if result.returncode:raise RuntimeError('Compilation failed: '+str(build/'compile.log'))
    print(json.dumps({'status':'compiled','receipt':str(build/'receipt.json')}))


if __name__=='__main__':main()
