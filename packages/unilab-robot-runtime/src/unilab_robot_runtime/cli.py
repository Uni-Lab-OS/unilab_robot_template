"""不经过 FE/Backend 的本地机械臂维护命令行入口。"""

from __future__ import annotations

import argparse
import importlib
import json
import sys
import uuid
from collections.abc import Callable, Sequence
from dataclasses import asdict, is_dataclass
from enum import Enum
from typing import Any, TextIO

from unilab_robot_contracts import (
    AngleUnit,
    CommissioningPoseInput,
    ControlledStopCommand,
    EulerRotationOrder,
    JointJogCommand,
    MotionDirection,
    MovePoseCommand,
    MoveTargetCommand,
    TcpAxis,
    TcpJogCommand,
)


def main(
    argv: Sequence[str] | None = None,
    *,
    runtime_factory: Callable[[], Any] | None = None,
    output: TextIO | None = None,
) -> int:
    """打开独占维护会话，执行一条命令并输出稳定 JSON。"""

    parser = _parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    factory = runtime_factory
    if factory is None:
        if not args.factory:
            parser.error("必须提供 --factory module:function")
        factory = _load_factory(args.factory)
    binding = factory()
    session = binding.open_maintenance_session(args.owner)
    destination = output or sys.stdout
    try:
        if args.operation == "snapshot":
            payload = _jsonable(session.snapshot())
        else:
            command = _command(args)
            result = session.execute(command)
            payload = _jsonable(result)
    finally:
        session.close()
        binding.close()
    destination.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
    return 0


def _parser() -> argparse.ArgumentParser:
    """创建不按 PLC/SDK/MoveIt 拆分的统一 CLI 参数树。"""

    parser = argparse.ArgumentParser(prog="unilab-robot-maintenance")
    parser.add_argument("--factory", help="返回 RuntimeBinding 的 module:function")
    parser.add_argument("--owner", required=True, help="维护会话所有者")
    commands = parser.add_subparsers(dest="operation", required=True)
    commands.add_parser("snapshot", help="读取当前关节/TCP 和 Fence 快照")
    move_target = commands.add_parser("move-target")
    _identity_arguments(move_target)
    move_target.add_argument("--target-ref", required=True)
    move_target.add_argument("--target-revision", required=True)
    move_pose = commands.add_parser("move-pose")
    _identity_arguments(move_pose)
    move_pose.add_argument("--xyz-mm", nargs=3, type=float, required=True)
    move_pose.add_argument("--rotation", nargs=3, type=float, required=True)
    move_pose.add_argument("--angle-unit", choices=[item.value for item in AngleUnit], required=True)
    move_pose.add_argument("--rotation-order", choices=[item.value for item in EulerRotationOrder], required=True)
    move_pose.add_argument("--tool-context-digest", required=True)
    tcp_jog = commands.add_parser("tcp-jog")
    _identity_arguments(tcp_jog)
    tcp_jog.add_argument("--frame-ref", choices=["arm_base", "tool"], required=True)
    tcp_jog.add_argument("--axis", choices=[item.value for item in TcpAxis], required=True)
    tcp_jog.add_argument("--direction", choices=[item.value for item in MotionDirection], required=True)
    tcp_jog.add_argument("--step-si", type=float, required=True)
    joint_jog = commands.add_parser("joint-jog")
    _identity_arguments(joint_jog)
    joint_jog.add_argument("--joint-ref", required=True)
    joint_jog.add_argument("--direction", choices=[item.value for item in MotionDirection], required=True)
    joint_jog.add_argument("--step-si", type=float, required=True)
    stop = commands.add_parser("stop")
    stop.add_argument("--command-id", default=uuid.uuid4().hex)
    stop.add_argument("--hardware-profile-digest", required=True)
    stop.add_argument("--source-boot-id", default=f"cli-{uuid.uuid4().hex}")
    stop.add_argument("--sequence", type=int, default=1)
    stop.add_argument("--target-command-id", required=True)
    stop.add_argument("--reason", required=True)
    return parser


def _identity_arguments(parser: argparse.ArgumentParser) -> None:
    """添加所有有限维护运动共享的身份与低速限制。"""

    parser.add_argument("--command-id", default=uuid.uuid4().hex)
    parser.add_argument("--hardware-profile-digest", required=True)
    parser.add_argument("--source-boot-id", default=f"cli-{uuid.uuid4().hex}")
    parser.add_argument("--sequence", type=int, default=1)
    parser.add_argument("--motion-profile-ref", default="maintenance-slow")
    parser.add_argument("--velocity-scale", type=float, default=0.05)
    parser.add_argument("--acceleration-scale", type=float, default=0.05)


def _command(args: argparse.Namespace) -> Any:
    """把 CLI 参数唯一映射为封闭调试命令联合。"""

    common = {
        "command_id": args.command_id,
        "hardware_profile_digest": args.hardware_profile_digest,
        "source_boot_id": args.source_boot_id,
        "monotonic_sequence": args.sequence,
    }
    if args.operation == "stop":
        return ControlledStopCommand(
            **common,
            target_command_id=args.target_command_id,
            reason=args.reason,
        )
    finite = {
        **common,
        "motion_profile_ref": args.motion_profile_ref,
        "velocity_scale": args.velocity_scale,
        "acceleration_scale": args.acceleration_scale,
    }
    if args.operation == "move-target":
        return MoveTargetCommand(
            **finite,
            target_ref=args.target_ref,
            target_revision=args.target_revision,
        )
    if args.operation == "move-pose":
        return MovePoseCommand(
            **finite,
            pose_input=CommissioningPoseInput(
                frame_ref="arm_base",
                xyz_mm=tuple(args.xyz_mm),
                rotation_xyz=tuple(args.rotation),
                angle_unit=AngleUnit(args.angle_unit),
                rotation_order=EulerRotationOrder(args.rotation_order),
            ),
            tool_context_digest=args.tool_context_digest,
        )
    if args.operation == "tcp-jog":
        return TcpJogCommand(
            **finite,
            frame_ref=args.frame_ref,
            axis=TcpAxis(args.axis),
            direction=MotionDirection(args.direction),
            step_si=args.step_si,
        )
    if args.operation == "joint-jog":
        return JointJogCommand(
            **finite,
            joint_ref=args.joint_ref,
            direction=MotionDirection(args.direction),
            step_si=args.step_si,
        )
    raise ValueError(f"未知维护命令: {args.operation}")


def _load_factory(reference: str) -> Callable[[], Any]:
    """加载显式 module:function，不执行任意字符串表达式。"""

    module_name, separator, function_name = reference.partition(":")
    if not separator or not module_name or not function_name:
        raise ValueError("--factory 必须使用 module:function")
    factory = getattr(importlib.import_module(module_name), function_name)
    if not callable(factory):
        raise TypeError("--factory 引用必须可调用")
    return factory


def _jsonable(value: Any) -> Any:
    """把合同数据类和枚举递归转换为 JSON 值。"""

    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    return value


if __name__ == "__main__":
    raise SystemExit(main())
