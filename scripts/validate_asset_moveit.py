"""Run real MoveItPy/FCL against an externally compiled, locked asset model."""
import argparse
import json
import math
from pathlib import Path
import traceback

import numpy as np
from unilab_robot_contracts.planning import PlanRequest, PlanGoal, CartesianGoal, CollisionWorld, CollisionObject, CollisionGeometry
from unilab_robot_runtime.asset_moveit import AssetMoveItPlanner, sha, source_coordinates
from unilab_robot_runtime.moveit_planner import MoveItPlanner, MoveItPlannerConfig


def run(folder):
    api = AssetMoveItPlanner(folder, expected_sha256=sha(folder/'model.json'), config=MoveItPlannerConfig(node_name='asset_validation', planning_time_s=15))
    robot, data, planner = api.robot, api.data, api.backend
    home = np.asarray(data['authored_q'])
    request = PlanRequest(robot, tuple(home), PlanGoal.from_joint(home))
    planner._ensure_robot(request)
    planner._load_world(request)
    rng = np.random.default_rng(1208)
    good = []
    for i in range(500):
        q = home if i == 0 else np.clip(home + rng.uniform(-1.0, 1.0, len(home)), np.array(robot.joint_limits)[:, 0]+1e-5, np.array(robot.joint_limits)[:, 1]-1e-5)
        try: source_coordinates(data, q)
        except ValueError: continue
        if not planner._collision_state(request, q)['collision']:
            good.append(q)
            if len(good) == 2: break
    if len(good) < 2: raise ValueError('No collision-free start and goal; no collision exclusions added')
    request = PlanRequest(robot, tuple(good[0]), PlanGoal.from_joint(good[1]), timeout_s=15)
    result = api.plan(request)
    for point in result.waypoints: source_coordinates(data, point.q)
    from unilab_robot_runtime.moveit_planner import _pose_matrix_to_message
    goal_transform=np.asarray(planner._state(request,good[1]).get_global_link_transform(robot.flange_link))
    native_pose=_pose_matrix_to_message(goal_transform); quat=native_pose.orientation
    pose_goal=CartesianGoal('world',tuple(goal_transform[:3,3]),(quat.x,quat.y,quat.z,quat.w),robot.flange_link)
    pose_request=PlanRequest(robot,tuple(good[0]),PlanGoal.from_pose(pose_goal),seeds=(tuple(good[0]),),timeout_s=15)
    pose_result=api.plan(pose_request)
    pose_error=None
    if pose_result.status=='success':
        actual=np.asarray(planner._state(pose_request,pose_result.waypoints[-1].q).get_global_link_transform(robot.flange_link))
        pose_error=float(np.max(np.abs(actual-goal_transform)))
    # Independent source A R(q) B^-1 FK versus actual MoveIt link transforms.
    errors=[]
    for q in good:
        state=planner._state(request, q)
        matrix=np.array(data['base_world'])
        for angle, frame in zip(q, data['joint_frames']):
            axis=np.asarray(frame['axis']); x,y,z=axis; c=math.cos(angle); s=math.sin(angle)
            cross=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            rot=np.eye(4); rot[:3,:3]=c*np.eye(3)+(1-c)*np.outer(axis,axis)+s*cross
            matrix=matrix @ np.array(frame['a']) @ rot @ np.linalg.inv(frame['b'])
            actual=np.asarray(state.get_global_link_transform(frame['child']))
            errors.append(float(np.abs(actual-matrix).max()))
    # A containing obstacle must be reported by FCL and rejected by planning.
    pose=np.eye(4)
    world=CollisionWorld(objects=(CollisionObject('blocking_box',tuple(map(tuple,pose)),(CollisionGeometry(kind='box',size_m=(5.,5.,5.)),)),))
    blocked=PlanRequest(robot, tuple(good[0]), PlanGoal.from_joint(good[1]), collision_world=world)
    planner._load_world(blocked)
    collision=api.check_collision(blocked, good[0])
    negative=api.plan(blocked)
    return {'model':data['model_id'],'asset':data['asset'], 'start_q':good[0].tolist(),'goal_q':good[1].tolist(),
            'nominal':result.as_mapping(), 'pose_plan':pose_result.as_mapping(),'pose_goal_matrix_error':pose_error,
            'fk_max_matrix_error':max(errors), 'fk_link_samples':len(errors),
            'blocking_collision':collision,'blocking_plan':negative.as_mapping(),
            'passed':result.status=='success' and pose_result.status=='success' and pose_error<1e-3 and max(errors)<1e-6 and collision['collision'] and negative.status!='success'}


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('folder'); p.add_argument('--receipt',required=True); a=p.parse_args()
    try: result=run(Path(a.folder))
    except Exception as exc: result={'passed':False,'error':str(exc),'trace':traceback.format_exc()}
    Path(a.receipt).write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k in ('passed','model','error','fk_max_matrix_error')},ensure_ascii=False),flush=True)
    raise SystemExit(0 if result['passed'] else 1)
