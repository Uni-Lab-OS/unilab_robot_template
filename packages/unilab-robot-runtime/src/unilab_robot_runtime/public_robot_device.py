"""OS-callable kinematic simulation driver for an explicitly locked public robot.

No controller connection is implemented here. A real MoveIt/FCL plan is consumed
into USD time samples; this is motion execution in simulation, not dynamics.
"""
from __future__ import annotations
import json
from pathlib import Path
import threading

from unilab_robot_contracts.public_catalog import resolve_public_robot
from unilab_robot_contracts.planning import PlanRequest, PlanGoal, CollisionWorld
from .asset_moveit import AssetMoveItPlanner, source_coordinates


class PublicRobotSimulationDevice:
    def __init__(self, *, catalog_uuid: str, library_root: str, compiled_model: str,
                 model_sha256: str, output_root: str, initial_q: list[float]):
        self.catalog, self.model = resolve_public_robot(catalog_uuid, library_root)
        self.uuid = catalog_uuid
        self.package = self.catalog.asset(self.model, library_root)
        self.planner = AssetMoveItPlanner(compiled_model, expected_sha256=model_sha256)
        if self.planner.data['asset'] != self.catalog.model(self.model)['asset']:
            raise ValueError('Compiled model belongs to a different locked asset')
        source_coordinates(self.planner.data, initial_q)
        self.q = tuple(map(float, initial_q)); self.output = Path(output_root).resolve()
        if self.output.is_relative_to(Path(library_root).resolve()):
            raise ValueError('Motion artifacts must be outside the asset library')
        self.output.mkdir(parents=True, exist_ok=True)
        self.world = CollisionWorld(); self._lock = threading.Lock(); self.counter = 0
        request = PlanRequest(self.planner.robot, self.q, PlanGoal.from_joint(self.q))
        if self.planner.check_collision(request, self.q)['collision']:
            raise ValueError('Initial simulation state is in collision')

    @property
    def joint_positions(self) -> list[float]:
        return list(self.q)

    def move_joint(self, q_goal: list[float]) -> dict:
        """Plan and execute into USD using a simulation clock; angles are radians."""
        if not self._lock.acquire(blocking=False):
            raise RuntimeError('Robot motion already in progress')
        try:
            request = PlanRequest(self.planner.robot, self.q, PlanGoal.from_joint(q_goal), collision_world=self.world)
            result = self.planner.plan(request)
            if result.status != 'success' or not result.waypoints:
                raise ValueError('Motion rejected: ' + json.dumps(result.as_mapping(), default=str))
            if max(abs(a-b) for a,b in zip(result.waypoints[0].q, self.q)) > 1e-6:
                raise ValueError('Unplanned bridge from current simulation state')
            # Check every emitted state before changing simulation state.
            for point in result.waypoints:
                source_coordinates(self.planner.data, point.q)
                if self.planner.check_collision(request, point.q)['collision']:
                    raise ValueError('Trajectory waypoint in collision')
            self.counter += 1
            folder = self.output / f'motion-{self.counter:04d}'
            folder.mkdir(exist_ok=False)
            motion = self._write_motion(result, folder)
            self.q = tuple(result.waypoints[-1].q)
            receipt = {'catalog_uuid': self.uuid, 'asset': self.planner.data['asset'],
                       'mode': 'usd_kinematic_simulation', 'hardware_connected': False,
                       'plan': result.as_mapping(), 'joint_positions': self.joint_positions, **motion}
            (folder/'receipt.json').write_text(json.dumps(receipt, ensure_ascii=False, indent=2), encoding='utf-8')
            return receipt
        finally:
            self._lock.release()

    def _write_motion(self, result, folder):
        import numpy as np
        from scipy.spatial.transform import Rotation
        from pxr import Usd, UsdGeom, UsdPhysics, Gf
        from unilab_robot_contracts._asset_package import verify, ADAPTER, read
        data = self.planner.data
        verify(self.package, data['asset']['package_digest'])
        native = read(self.package/ADAPTER)
        source = self.package/native['entrypoints']['usd']
        stage = Usd.Stage.CreateNew(str(folder/'motion.usda'))
        stage.GetRootLayer().subLayerPaths = [source.as_posix()]
        stage.SetTimeCodesPerSecond(1.0)
        stage.SetStartTimeCode(0); stage.SetEndTimeCode(result.waypoints[-1].time_s)
        # Source joints identify exact bodies, without depending on label order.
        joints = [UsdPhysics.Joint(stage.GetPrimAtPath(f['source_joint'])) for f in data['joint_frames']]
        paths = [joints[0].GetBody0Rel().GetTargets()[0]] + [j.GetBody1Rel().GetTargets()[0] for j in joints]
        ops = []
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            xform = UsdGeom.Xformable(prim); xform.ClearXformOpOrder()
            xform.SetResetXformStack(True)
            ops.append(xform.AddTransformOp(opSuffix='osMotion'))
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
        for point in result.waypoints:
            matrix = np.array(data['base_world']); ops[0].Set(Gf.Matrix4d(matrix.T.tolist()), point.time_s)
            for i, (angle, frame) in enumerate(zip(point.q, data['joint_frames'])):
                rot = np.eye(4); rot[:3,:3] = Rotation.from_rotvec(np.array(frame['axis'])*angle).as_matrix()
                matrix = matrix @ np.array(frame['a']) @ rot @ np.linalg.inv(frame['b'])
                ops[i+1].Set(Gf.Matrix4d(matrix.T.tolist()), point.time_s)
        stage.GetRootLayer().Save()
        return {'motion_usd': str(folder/'motion.usda'), 'frames': len(result.waypoints),
                'duration_s': result.waypoints[-1].time_s, 'source_body_paths': list(map(str, paths))}

    def close(self):
        self.planner.backend.close()
