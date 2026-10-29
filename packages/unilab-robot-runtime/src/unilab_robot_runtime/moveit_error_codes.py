"""``moveit_msgs/MoveItErrorCodes`` 的显式映射。

数值与官方名称取自本机 MoveIt 2.5.9 安装的两份权威定义：

* ``Library/share/moveit_msgs/msg/MoveItErrorCodes.msg``
* ``Library/include/moveit/utils/moveit_error_code.h``

历史实现把几个码记错了，而错误码又是判断"碰撞 / 不可达 / 超时"的唯一依据，
所以这里集中成一张表，``moveit_planner``（MoveItPy）与 ``moveit_move_group``
（ROS 2 服务）都必须查这张表，不再各自维护：

* ``-2`` 是 ``INVALID_MOTION_PLAN``（planning_pipeline 复检路径无效时也会报它），
  不是"起始态/目标态碰撞"；
* ``-16`` 是 ``INVALID_GOAL_CONSTRAINTS``，不是超时；
* ``-31`` 是 ``NO_IK_SOLUTION``；
* ``-1`` 是 ``PLANNING_FAILED``；``FAILURE`` 是 ``99999``；
* ``-26``/``-27``/``-28`` 是 ``START_STATE_INVALID`` / ``GOAL_STATE_INVALID`` /
  ``UNRECOGNIZED_GOAL_TYPE``，``CRASH`` / ``ABORT`` 才是 ``-29`` / ``-30``。

``PlanStatus`` 是合同里的封闭字面量
（``success | invalid | collision | unreachable | timeout``），因此这里把
"规划器为什么失败"放在 ``category``（写进 ``PlanResult.diagnostics
["failure_category"]``），把"对调用方意味着什么"放在 ``status``；原始错误码
（``moveit_error_code``）和 MoveIt 官方名称（``moveit_error_name``）始终保留，
不因为映射而丢失。
"""

from __future__ import annotations

from dataclasses import dataclass

from unilab_robot_contracts.planning import PlanStatus

SUCCESS_CODE = 1

#: 无法读出的错误码（``None``、对象、字符串）用的占位符。
UNREADABLE_MOVEIT_ERROR = "UNREADABLE_MOVEIT_ERROR"
#: 表里没有的整数错误码用的官方名称占位符。
UNRECOGNIZED_MOVEIT_ERROR = "UNRECOGNIZED_MOVEIT_ERROR"


@dataclass(frozen=True, slots=True)
class MoveItErrorCode:
    """一个 MoveIt 错误码的完整解释。

    ``code`` 为原始整数（不可读时为 ``None``），``name`` 是 MoveIt 官方名称，
    ``status`` 是合同状态，``category`` 是稳定的机器可读失败类别。
    """

    code: int | None
    name: str
    status: PlanStatus
    category: str
    message: str


def _entry(code: int, name: str, status: PlanStatus, category: str, message: str) -> MoveItErrorCode:
    return MoveItErrorCode(code=code, name=name, status=status, category=category, message=message)


#: ``MoveItErrorCodes`` 全表；顺序与 msg 文件一致。
MOVEIT_ERROR_CODES: dict[int, MoveItErrorCode] = {
    1: _entry(1, "SUCCESS", "success", "success", "规划成功"),
    -1: _entry(-1, "PLANNING_FAILED", "unreachable", "failure",
               "规划失败，MoveIt 未给出更细原因"),
    -2: _entry(-2, "INVALID_MOTION_PLAN", "invalid", "invalid_motion_plan",
               "运动计划无效（规划器返回值或管线复检判定路径无效）"),
    -3: _entry(-3, "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE", "collision",
               "motion_plan_invalidated_by_environment_change", "环境变化使已规划路径失效"),
    -4: _entry(-4, "CONTROL_FAILED", "invalid", "control_failed", "控制器执行失败"),
    -5: _entry(-5, "UNABLE_TO_AQUIRE_SENSOR_DATA", "invalid",
               "unable_to_acquire_sensor_data", "无法取得传感器数据（msg 原文拼写）"),
    -6: _entry(-6, "TIMED_OUT", "timeout", "timed_out", "规划或执行超时"),
    -7: _entry(-7, "PREEMPTED", "invalid", "preempted", "规划或执行被抢占"),
    -10: _entry(-10, "START_STATE_IN_COLLISION", "collision",
                "start_state_in_collision", "起始状态碰撞"),
    -11: _entry(-11, "START_STATE_VIOLATES_PATH_CONSTRAINTS", "invalid",
                "start_state_violates_path_constraints", "起始状态违反路径约束"),
    -12: _entry(-12, "GOAL_IN_COLLISION", "collision", "goal_in_collision", "目标状态碰撞"),
    -13: _entry(-13, "GOAL_VIOLATES_PATH_CONSTRAINTS", "collision",
                "goal_violates_path_constraints", "目标状态违反路径约束"),
    -14: _entry(-14, "GOAL_CONSTRAINTS_VIOLATED", "invalid", "goal_constraints_violated",
                "目标状态不满足约束"),
    -15: _entry(-15, "INVALID_GROUP_NAME", "invalid", "invalid_group_name", "规划组名称无效"),
    -16: _entry(-16, "INVALID_GOAL_CONSTRAINTS", "invalid", "invalid_goal_constraints",
                "目标约束无效（规划器无法解释该目标）"),
    -17: _entry(-17, "INVALID_ROBOT_STATE", "invalid", "invalid_robot_state", "机器人状态无效"),
    -18: _entry(-18, "INVALID_LINK_NAME", "invalid", "invalid_link_name", "连杆名称无效"),
    -19: _entry(-19, "INVALID_OBJECT_NAME", "invalid", "invalid_object_name", "碰撞对象名称无效"),
    -21: _entry(-21, "FRAME_TRANSFORM_FAILURE", "invalid", "frame_transform_failure",
                "坐标变换失败"),
    -22: _entry(-22, "COLLISION_CHECKING_UNAVAILABLE", "invalid",
                "collision_checking_unavailable", "碰撞检测不可用"),
    -23: _entry(-23, "ROBOT_STATE_STALE", "invalid", "robot_state_stale", "机器人状态过期"),
    -24: _entry(-24, "SENSOR_INFO_STALE", "invalid", "sensor_info_stale", "传感器信息过期"),
    -25: _entry(-25, "COMMUNICATION_FAILURE", "invalid", "communication_failure", "通信失败"),
    -26: _entry(-26, "START_STATE_INVALID", "invalid", "start_state_invalid", "起始状态无效"),
    -27: _entry(-27, "GOAL_STATE_INVALID", "invalid", "goal_state_invalid", "目标状态无效"),
    -28: _entry(-28, "UNRECOGNIZED_GOAL_TYPE", "invalid", "unrecognized_goal_type",
                "无法识别的目标类型"),
    -29: _entry(-29, "CRASH", "invalid", "crash", "MoveIt 侧异常退出"),
    -30: _entry(-30, "ABORT", "invalid", "abort", "MoveIt 主动中止"),
    -31: _entry(-31, "NO_IK_SOLUTION", "unreachable", "no_ik_solution",
                "没有可用的逆运动学解"),
    99999: _entry(99999, "FAILURE", "invalid", "failure", "MoveIt 通用失败"),
}


def describe_moveit_error(code: object) -> MoveItErrorCode:
    """把任意来源的错误码解释成 ``MoveItErrorCode``。

    可读整数走全表；表里没有的整数保留原值并标 ``UNRECOGNIZED_MOVEIT_ERROR``；
    ``None``、布尔值、无法转成整数的对象标 ``UNREADABLE_MOVEIT_ERROR`` 且
    ``code=None``——两者都不抛异常，规划失败路径不允许再引发二次异常。
    """

    if isinstance(code, bool):
        return MoveItErrorCode(
            code=None, name=UNREADABLE_MOVEIT_ERROR, status="invalid",
            category="moveit_error_unreadable", message="MoveIt 返回的错误码不可读")
    try:
        value = int(code)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return MoveItErrorCode(
            code=None, name=UNREADABLE_MOVEIT_ERROR, status="invalid",
            category="moveit_error_unreadable", message="MoveIt 返回的错误码不可读")
    known = MOVEIT_ERROR_CODES.get(value)
    if known is not None:
        return known
    return MoveItErrorCode(
        code=value, name=UNRECOGNIZED_MOVEIT_ERROR, status="invalid",
        category=f"moveit_error_{value}", message="未识别的 MoveIt 错误码")


def status_for_moveit_error(code: object) -> PlanStatus:
    """只取合同状态；``moveit_planner`` 的历史入口保留此包装。"""

    return describe_moveit_error(code).status


def error_code_fields(code: object) -> dict[str, object]:
    """``PlanResult.diagnostics`` 里的错误码字段；原始码与官方名称都不丢。"""

    spec = describe_moveit_error(code)
    return {
        "moveit_error_code": spec.code,
        "moveit_error_name": spec.name,
        "failure_category": spec.category,
    }


__all__ = [
    "MOVEIT_ERROR_CODES",
    "SUCCESS_CODE",
    "UNREADABLE_MOVEIT_ERROR",
    "UNRECOGNIZED_MOVEIT_ERROR",
    "MoveItErrorCode",
    "describe_moveit_error",
    "error_code_fields",
    "status_for_moveit_error",
]
