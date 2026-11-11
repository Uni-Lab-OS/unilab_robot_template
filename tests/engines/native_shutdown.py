"""Native Linux MoveIt startup/shutdown regression; no hardware or ROS client.

Run from an installed robotics/ROS environment:
 python native_shutdown.py --output /tmp/shutdown-check --runs 3
 --unpatched is a diagnostic negative control, never a production session flag.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import yaml
from unilab_robot_runtime.ros_environment import move_group_environment


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--prefix',type=Path)
    parser.add_argument('--robot-config',type=Path)
    parser.add_argument('--runs',type=int,default=3)
    parser.add_argument('--domain-id',type=int,default=96)
    parser.add_argument('--unpatched',action='store_true')
    args=parser.parse_args()
    if not sys.platform.startswith('linux'):
        parser.error('This native regression targets the diagnosed Linux ELF plugin lifecycle')
    if args.runs<1:parser.error('--runs must be positive')
    if args.prefix is None:
        from ament_index_python.packages import get_package_prefix
        args.prefix=Path(get_package_prefix('moveit_ros_move_group'))
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    executable=args.prefix/'lib/moveit_ros_move_group/move_group'
    if args.robot_config:
        config=args.robot_config.read_text()
    else:
        params={
            'robot_description':'<robot name="minimal"><link name="base"/><link name="tip"/>'
              '<joint name="axis" type="revolute"><parent link="base"/><child link="tip"/>'
              '<axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint></robot>',
            'robot_description_semantic':'<robot name="minimal"><group name="arm"><joint name="axis"/></group></robot>',
            'allow_trajectory_execution':False,'planning_pipelines':['ompl'],
            'default_planning_pipeline':'ompl','ompl':{'planning_plugins':['ompl_interface/OMPLPlanner']}}
        config=yaml.safe_dump({'/**':{'ros__parameters':params}})
    (out/'move-group.yaml').write_text(config)
    env=move_group_environment(args.prefix,domain_id=args.domain_id)
    if args.unpatched:
        # Restore the caller's original loader setting for an explicit control.
        env.pop('LD_PRELOAD',None)
        if 'LD_PRELOAD' in os.environ:env['LD_PRELOAD']=os.environ['LD_PRELOAD']
    versions={}
    for package in ('moveit_ros_move_group','rclcpp'):
        manifest=args.prefix/'share'/package/'package.xml'
        if manifest.is_file():versions[package]=ET.parse(manifest).getroot().findtext('version')
    report={'hardware':False,'scope':'native shutdown only; no planning or world geometry',
            'unpatched':args.unpatched,'versions':versions,
            'executable':str(executable),'executable_sha256':hashlib.sha256(executable.read_bytes()).hexdigest(),
            'configuration_sha256':hashlib.sha256(config.encode()).hexdigest(),
            'ld_preload':env.get('LD_PRELOAD'),'runs':[]}
    for index in range(args.runs):
        case=out/str(index+1);case.mkdir()
        ready=False;forced=False
        with (case/'native.log').open('w') as log:
            process=subprocess.Popen([str(executable),'--ros-args','--params-file',str(out/'move-group.yaml')],
                stdout=log,stderr=subprocess.STDOUT,env=env)
            try:
                deadline=time.monotonic()+30
                while time.monotonic()<deadline and process.poll() is None:
                    if 'You can start planning now!' in (case/'native.log').read_text(errors='replace'):
                        ready=True;break
                    time.sleep(.1)
                if process.poll() is None:
                    (case/'maps-before-shutdown.txt').write_text(Path(f'/proc/{process.pid}/maps').read_text())
                    process.send_signal(signal.SIGTERM)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    forced=True;process.kill();process.wait(timeout=10)
            finally:
                if process.poll() is None:process.kill();process.wait(timeout=10)
        result={'ready':ready,'returncode':process.returncode,'forced_kill':forced,
                'clean_shutdown':ready and process.returncode==0 and not forced}
        (case/'result.json').write_text(json.dumps(result,indent=2))
        report['runs'].append(result)
        report['passed']=all(row['clean_shutdown'] for row in report['runs'])
        (out/'result.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(result),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
