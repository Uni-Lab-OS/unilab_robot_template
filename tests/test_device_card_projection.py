"""机械臂卡片 debug snapshot 与 FK 投影（template 通用层）。"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from unilab_robot_runtime.device_card.context import ArmCardContext
from unilab_robot_runtime.device_card.manual_motion import read_debug_snapshot


class _Snapshot:
    source = "test:moveit"
    online = True
    idle = True
    observed_at = 1.0
    execution_fenced = False
    active_command_id = None
    tcp_pose = SimpleNamespace(
        frame_ref="arm_base",
        xyz_m=(0.999, 0.999, 0.999),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    joint_positions = tuple(
        SimpleNamespace(joint_ref=f"joint_{index}", position_si=0.0)
        for index in range(1, 7)
    )

    def is_fresh(self) -> bool:
        return True


class _Port:
    commissioning_target_revision = "test-point-set@1.0.0"
    commissioning_velocity_limit = 0.2
    commissioning_acceleration_limit = 0.2
    commissioning_capabilities = SimpleNamespace(
        move_target=True,
        move_pose=True,
        tcp_jog=True,
        joint_jog=True,
        controlled_stop=True,
    )

    def __init__(self, catalog: tuple[SimpleNamespace, ...]) -> None:
        self.commissioning_target_catalog = catalog

    def commissioning_snapshot(self) -> _Snapshot:
        return _Snapshot()


class _Binding:
    def __init__(self, port: _Port) -> None:
        self.commissioning_port = port


class _RailAxis:
    qualified_joint_names = ("test_rail_joint",)

    def __init__(self, position_si: float = 0.42) -> None:
        self.position_si = float(position_si)

    def joint_state_frame(self) -> tuple[tuple[str, ...], tuple[float, ...]]:
        return self.qualified_joint_names, (self.position_si,)

    def move_to_si(self, position_si: float) -> None:
        self.position_si = float(position_si)


def _catalog_entry() -> SimpleNamespace:
    return SimpleNamespace(
        target_ref="warehouse.S081_pour_tilt",
        source_point="S081_pour_tilt",
        kind="joint_positions",
        editable=True,
        joint_positions_si=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6),
        rail_position_si=0.4321,
        group_ref="warehouse",
    )


def _context(*, rail_axis: _RailAxis | None = None) -> ArmCardContext:
    def project_target(entry: object) -> dict[str, object]:
        return {
            "target_ref": str(getattr(entry, "target_ref", "")),
            "source_point": str(getattr(entry, "source_point", "")),
            "kind": str(getattr(entry, "kind", "")),
            "editable": bool(getattr(entry, "editable", False)),
            "joint_positions_deg": [10.0, 20.0, 30.0, 40.0, 50.0, 60.0],
            "rail_position_mm": 432.1,
            "group_ref": str(getattr(entry, "group_ref", "")),
            "tcp_pose": {
                "frame_ref": "arm_base",
                "xyz_mm": [111.0, 222.0, 333.0],
                "rotation_xyz_deg": [0.0, 0.0, 0.0],
            },
        }

    def rail_snapshot(_binding: object, _port: object) -> tuple[dict[str, float] | None, object | None]:
        if rail_axis is None:
            return None, None
        _joint_names, rail_positions = rail_axis.joint_state_frame()
        return (
            {
                "position_mm": float(rail_positions[0]) * 1000.0,
                "travel_min_mm": 0.0,
                "travel_max_mm": 2250.0,
            },
            rail_axis,
        )

    return ArmCardContext(
        boot_id_prefix="test",
        project_target=project_target,
        rail_snapshot=rail_snapshot,
    )


def test_debug_snapshot_exposes_registered_parent_rail() -> None:
    axis = _RailAxis()
    port = _Port((_catalog_entry(),))
    snapshot = read_debug_snapshot(_Binding(port), _context(rail_axis=axis))

    assert snapshot["rail"] == {
        "position_mm": pytest.approx(420.0),
        "travel_min_mm": pytest.approx(0.0),
        "travel_max_mm": pytest.approx(2250.0),
    }
    assert snapshot["capabilities"]["rail_move"] is True


def test_card_catalog_projects_target_tcp_from_target_joints() -> None:
    port = _Port((_catalog_entry(),))
    target = read_debug_snapshot(_Binding(port), _context())["point_targets"][0]

    assert target["rail_position_mm"] == pytest.approx(432.1)
    assert target["tcp_pose"]["frame_ref"] == "arm_base"
    assert target["tcp_pose"]["xyz_mm"] == [111.0, 222.0, 333.0]
    assert target["tcp_pose"]["xyz_mm"] != [999.0, 999.0, 999.0]
