"""Real OS wrapper -> JSON driver dispatch -> MoveIt/FCL -> USD motion acceptance.

Run in ROS Jazzy with the actual OS installed and no hardware network access.
Requires a preceding validate_asset_moveit.py receipt for collision-free fixtures.
"""
import argparse
import hashlib
import json
from pathlib import Path
import traceback


def run(root, brand):
    import numpy as np
    import rclpy
    from pxr import Usd, UsdGeom
    from unilabos.config.config import BasicConfig
    from unilabos.resources.resource_tracker import ResourceDictInstance, JSON_UNILABOS_PARAM
    from unilabos.ros.nodes.base_device_node import ROS2DeviceNode
    from unilabos.utils.import_manager import get_class
    from unilab_robot_contracts.planning import PlanRequest, PlanGoal, CollisionWorld, CollisionObject, CollisionGeometry
    from unilab_robot_runtime.asset_moveit import sha
    fixture = json.loads((root/'engine'/f'{brand}.json').read_bytes())
    assert fixture['passed']
    row = next(r for r in json.loads((root/'selected.json').read_bytes()) if r['brand'] == brand)
    output = root/'os-motion'/brand; output.mkdir(parents=True, exist_ok=False)
    BasicConfig.is_host_mode = True
    rclpy.init()
    config = ResourceDictInstance.get_resource_instance_from_dict({
        'id': 'robot_'+brand, 'name': 'robot_'+brand, 'uuid': row['uuid'],
        'class': 'robot.public_asset_simulation', 'type': 'device', 'config': {}, 'data': {}, 'extra': {}})
    driver_path = 'unilab_robot_runtime.public_robot_device:PublicRobotSimulationDevice'
    params = {'catalog_uuid': row['uuid'], 'library_root': str(root/'library'),
              'compiled_model': str(root/'models'/brand),
              'model_sha256': sha(root/'models'/brand/'model.json'),
              'output_root': str(output), 'initial_q': fixture['start_q']}
    (output/'os-device-config.json').write_text(json.dumps({'class': driver_path, 'params': params}, indent=2))
    wrapper = ROS2DeviceNode(device_id='robot_'+brand, device_uuid=row['uuid'],
        driver_class=get_class(driver_path), device_config=config, driver_params=params,
        status_types={}, action_value_mappings={'auto-move_joint': {'method_name': 'move_joint'}},
        hardware_interface={}, print_publish=False)
    driver = wrapper.driver_instance; node = wrapper.ros_node_instance
    calls = []; executions = []; errors = []
    def dispatch(q):
        command = {'function_name': 'auto-move_joint', 'function_args': {'q_goal': q}, JSON_UNILABOS_PARAM: {}}
        calls.append(command)
        return node._execute_driver_command(json.dumps(command), _execution_start=lambda: executions.append(len(calls)))
    try:
        forward = dispatch(fixture['goal_q'])
        assert np.max(np.abs(np.array(driver.joint_positions)-fixture['goal_q'])) < 1e-5
        assert np.max(np.abs(np.array(fixture['start_q'])-driver.joint_positions)) > 1e-3
        # Reopen USD and independently compare actual rendered body transforms
        # with native MoveIt FK at start/middle/end of the executed trajectory.
        stage = Usd.Stage.Open(forward['motion_usd'])
        points = forward['plan']['waypoints']; data = driver.planner.data
        request = PlanRequest(driver.planner.robot, tuple(fixture['start_q']), PlanGoal.from_joint(fixture['goal_q']))
        for point in (points[0], points[len(points)//2], points[-1]):
            state = driver.planner.backend._state(request, point['q'])
            cache = UsdGeom.XformCache(Usd.TimeCode(point['time_s']))
            for path, frame in zip(forward['source_body_paths'][1:], data['joint_frames']):
                usd = np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T
                native = np.array(state.get_global_link_transform(frame['child']))
                errors.append(float(np.abs(usd-native).max()))
        assert max(errors) < 1e-6
        backward = dispatch(fixture['start_q'])
        assert np.max(np.abs(np.array(driver.joint_positions)-fixture['start_q'])) < 1e-5
        before = driver.joint_positions; counter = driver.counter
        driver.world = CollisionWorld(objects=(CollisionObject('blocking_box', tuple(map(tuple,np.eye(4))),
            (CollisionGeometry(kind='box', size_m=(5.,5.,5.)),)),))
        try: dispatch(fixture['goal_q'])
        except ValueError as exc: blocked = str(exc)
        else: raise AssertionError('OS dispatched blocked motion')
        assert driver.joint_positions == before and driver.counter == counter
        driver.world = CollisionWorld()
        try: dispatch([999.]*len(before))
        except ValueError as exc: invalid = str(exc)
        else: raise AssertionError('OS dispatched out-of-limit motion')
        assert driver.joint_positions == before and driver.counter == counter
        result = {'passed': True, 'brand': brand, 'catalog_uuid': row['uuid'],
            'scope': 'actual OS device creation and JSON command dispatch; MoveIt/FCL and USD kinematic execution; no hardware or scheduler workflow',
            'driver_class': driver_path, 'os_driver_calls': calls, 'execution_starts': executions,
            'forward': forward, 'backward': backward, 'usd_fk_max_error': max(errors),
            'usd_fk_samples': len(errors), 'blocked_rejected': blocked, 'limit_rejected': invalid,
            'os_source_sha256': hashlib.sha256(Path(__import__('unilabos.ros.nodes.base_device_node',fromlist=['x']).__file__).read_bytes()).hexdigest()}
        return result
    finally:
        driver.close()
        node._executor.shutdown(wait=True)
        rclpy.get_global_executor().remove_node(node)
        node.destroy_node()
        loop = ROS2DeviceNode.get_asyncio_loop()
        if loop and loop.is_running(): loop.call_soon_threadsafe(loop.stop)
        if ROS2DeviceNode._asyncio_loop_thread: ROS2DeviceNode._asyncio_loop_thread.join(timeout=5)
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--root', type=Path, required=True); p.add_argument('--brand', required=True)
    a = p.parse_args()
    try: result = run(a.root.resolve(), a.brand)
    except Exception as exc: result = {'passed': False, 'brand': a.brand, 'error': str(exc), 'trace': traceback.format_exc()}
    (a.root/'engine'/f'{a.brand}-os.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k in ('passed','brand','error','usd_fk_max_error')}, ensure_ascii=False), flush=True)
    raise SystemExit(0 if result['passed'] else 1)
