"""Portable launch environment for an installed RoboStack prefix."""
import os
import sys
from pathlib import Path

def move_group_environment(prefix,*,domain_id,base=None):
    prefix=Path(prefix);env=dict(os.environ if base is None else base)
    library=prefix/"Library"
    if library.is_dir():
        # Some RoboStack DLLs install in lib rather than bin (notably tf2_eigen_kdl).
        search=[prefix,library/"bin",library/"lib",prefix/"Scripts"]
        separator=";"
        env["AMENT_PREFIX_PATH"]=str(library)
    else:
        search=[prefix/"bin"];separator=os.pathsep
        env["AMENT_PREFIX_PATH"]=str(prefix)
    env["PATH"]=separator.join(map(str,search))+separator+env.get("PATH","")
    env.update(ROS_DOMAIN_ID=str(int(domain_id)),ROS_LOCALHOST_ONLY="1",
               RMW_IMPLEMENTATION="rmw_fastrtps_cpp",PYTHONUTF8="1")
    # MoveIt 2.12.4 unloads the default-capability DSO before NodeBase releases
    # expired callback weak_ptr control blocks instantiated in that DSO. Keep
    # its code mapped until the native process exits (ELF loader ownership).
    # Only the Popen child consumes this variable; never mutate the ROS client
    # environment or bypass native destructors/shutdown status.
    manifest=prefix/"share/moveit_ros_move_group/package.xml"
    if sys.platform.startswith("linux") and manifest.is_file():
        import xml.etree.ElementTree as ET
        version=ET.parse(manifest).getroot().findtext("version", "").strip()
        if version=="2.12.4":
            library=prefix/"lib/libmoveit_move_group_default_capabilities.so"
            if not library.is_file():
                raise RuntimeError(f"MoveIt shutdown capability library missing: {library}")
            value=str(library.resolve())
            if any(c.isspace() for c in value) or ":" in value:
                raise RuntimeError("MoveIt capability library path cannot be represented by LD_PRELOAD")
            previous=env.get("LD_PRELOAD", "")
            entries=previous.replace(":", " ").split()
            if value not in entries:
                env["LD_PRELOAD"]=value+(" "+previous if previous else "")
    return env

from contextlib import contextmanager


@contextmanager
def native_planning_session(robot, rail_axis, output, *, domain_id=91):
    """在独占 ROS domain 启动/关闭本次拥有的原生规划进程，保留日志。"""
    import json, subprocess, sys, time
    import yaml
    from .moveit_move_group import MoveGroupPlanner, MoveGroupPlannerConfig, build_move_group_parameters
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    # Native simplification removes single-axis RRT reversals before TOTG;
    # the returned timed path still passes the full-scene dense validator.
    config=MoveGroupPlannerConfig(domain_id=domain_id,service_timeout_s=300,service_wait_s=90,simplify_solutions=True)
    params=build_move_group_parameters(robot,config,rail_axis=rail_axis)
    params["allow_trajectory_execution"]=False
    params["planning_pipelines"]=["ompl","pilz"]
    params["ompl"][robot.planning_group]["enforce_joint_model_state_space"]=False
    path=output/"move-group.yaml"
    path.write_text(yaml.safe_dump({"/**":{"ros__parameters":params}},sort_keys=False),encoding="utf8")
    prefix=Path(sys.prefix)
    try:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        prefix=Path(get_package_prefix('moveit_ros_move_group'))
        if os.name=='nt' and prefix.name.lower()=='library':
            prefix=prefix.parent  # RoboStack ament prefix already includes Library.
        import xml.etree.ElementTree as ET
        share=Path(get_package_share_directory('moveit_ros_planning'))
        modern=share/'default_response_adapters_plugin_description.xml'
        if modern.exists():
            classes={x.get('name') for x in ET.parse(modern).iter('class')}
            adapter='default_planning_response_adapters/AddTimeOptimalParameterization'
            if adapter not in classes:raise RuntimeError('Native response timing adapter missing')
            params['ompl']['request_adapters']=['default_planning_request_adapters/ResolveConstraintFrames',
                'default_planning_request_adapters/ValidateWorkspaceBounds',
                'default_planning_request_adapters/CheckStartStateBounds',
                'default_planning_request_adapters/CheckStartStateCollision']
            params['ompl']['response_adapters']=[adapter]
            # Jazzy's native orientation constraints use reference-frame error axes.
            from dataclasses import replace
            config=replace(config,native_orientation_error_frame='reference')
            path.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}},sort_keys=False),encoding='utf8')
    except ImportError:
        pass
    executable=(prefix/"Library/lib/moveit_ros_move_group/move_group.exe"
                if os.name=="nt" else prefix/"lib/moveit_ros_move_group/move_group")
    loaded=sys.modules.get("rclpy")
    if loaded is not None and loaded.ok():
        raise RuntimeError("Owned planning session requires an uninitialized ROS client")
    environment=move_group_environment(prefix,domain_id=domain_id)
    keys=("AMENT_PREFIX_PATH","PATH","ROS_DOMAIN_ID","ROS_LOCALHOST_ONLY","RMW_IMPLEMENTATION","PYTHONUTF8")
    previous={key:os.environ.get(key) for key in keys}
    from contextlib import ExitStack
    def restore_environment():
        for key,value in previous.items():
            if value is None:os.environ.pop(key,None)
            else:os.environ[key]=value
    def stop_native(process):
        original_error=sys.exc_info()[1]
        forced_kill=False
        termination_requested=process.poll() is None
        if termination_requested:process.terminate()
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            forced_kill=True
            process.kill();process.wait(timeout=10)
        clean_shutdown=process.returncode==0 and not forced_kill
        evidence=output/"native-exit.json"
        evidence.write_text(json.dumps(dict(
            pid=process.pid,returncode=process.returncode,closed_by_owner=True,
            termination_requested=termination_requested,forced_kill=forced_kill,
            clean_shutdown=clean_shutdown),indent=2),encoding="utf8")
        if not clean_shutdown:
            message=f"move_group shutdown failed (exit {process.returncode}); see {evidence}"
            if original_error is not None:
                original_error.add_note(message)
            else:
                raise RuntimeError(message)
    with ExitStack() as stack:
        stack.callback(restore_environment)
        os.environ.update({key:environment[key] for key in keys})
        if os.name=="nt":
            for directory in (prefix/"Library/bin",prefix/"Library/lib"):
                if directory.is_dir():
                    stack.callback(os.add_dll_directory(str(directory)).close)
        log=stack.enter_context((output/"move-group.log").open("w",encoding="utf8"))
        process=subprocess.Popen([str(executable),"--ros-args","--params-file",str(path)],
            stdout=log,stderr=subprocess.STDOUT,env=environment)
        stack.callback(stop_native,process)
        planner=MoveGroupPlanner(config)
        stack.callback(planner.close)
        if not planner.wait_ready():
            raise RuntimeError("Owned move_group services did not become ready")
        (output/"native-session.json").write_text(json.dumps(dict(
            pid=process.pid,domain_id=domain_id,model_digest=robot.digest,
            started_at_unix_s=time.time(),hardware=False,
            native_ld_preload=environment.get("LD_PRELOAD"),executable=str(executable)),indent=2),encoding="utf8")
        yield planner
