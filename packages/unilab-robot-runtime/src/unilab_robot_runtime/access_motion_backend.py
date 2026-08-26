"""把 D9-1S 机械臂、夹爪和负载观测收敛到一个执行边界。"""

from __future__ import annotations

import time
from collections.abc import Mapping
from dataclasses import replace
from threading import RLock
from typing import Any, Protocol

from unilab_robot_contracts import (
    BackendStatus,
    CommandRejectedError,
    CommandResult,
    CommandState,
    DispatchUnknownError,
    EndEffectorPort,
    MotionSegment,
    ObservationState,
    RobotCommand,
    RobotExecutionBackend,
    ToolChangerPort,
    ToolContext,
    WorkCellPhaseKind,
)


class PayloadPlanningScenePort(Protocol):
    """机械臂运行时消费的最小持料碰撞场景端口。"""

    def attach_payload(
        self,
        *,
        command_id: str,
        payload_instance_ref: str,
        payload_profile_ref: str,
        attachment_generation: int,
    ) -> Mapping[str, Any]:
        """夹爪确认持料后挂载碰撞体并返回回读收据。"""

    def detach_payload(
        self,
        *,
        command_id: str,
        payload_instance_ref: str,
        payload_profile_ref: str,
        attachment_generation: int,
    ) -> Mapping[str, Any]:
        """夹爪确认释放后解除同一碰撞体。"""

    def snapshot(self) -> list[Mapping[str, Any]]:
        """返回当前已经严格回读确认的持料碰撞体。"""

    def finalize_detached_payload(
        self,
        *,
        command_id: str,
        payload_instance_ref: str,
        payload_profile_ref: str,
        attachment_generation: int,
    ) -> Mapping[str, Any]:
        """机械臂撤离完成后撤销临时碰撞许可并保留世界物料。"""

    def settle_unknown_payload_state(
        self,
        *,
        command_id: str,
        holding_payload: bool,
    ) -> Mapping[str, Any] | None:
        """在 UNKNOWN 人工结算时对账夹爪和场景，不得派发物理动作。"""


class AccessMotionBackend:
    """执行取放接近块或保持持料的纯 Arm 倒液块。"""

    def __init__(
        self,
        *,
        arm_backend: RobotExecutionBackend,
        end_effector: EndEffectorPort,
        tool_changer: ToolChangerPort,
        expected_tool_context: ToolContext,
        payload_planning_scene: PayloadPlanningScenePort | None = None,
        require_payload_collision: bool = False,
    ) -> None:
        """绑定机械臂后端和独立夹爪端口；二者均由部署组合根选择。"""

        self.arm_backend = arm_backend
        self.end_effector = end_effector
        self.tool_changer = tool_changer
        self.expected_tool_context = expected_tool_context
        self.payload_planning_scene = payload_planning_scene
        self.require_payload_collision = bool(require_payload_collision)
        self.endpoint_ids = arm_backend.endpoint_ids
        self._results: dict[str, CommandResult] = {}
        self._active_arm_command_id: str | None = None
        self._active_lock = RLock()
        # Runtime 可在 ROS Executor 启动前装配，构造阶段不得读取硬件服务。
        # validate_before_dispatch/execute 会在任何 rail/arm 物理作用前完成对账。
        self._payload_scene_state_ready = payload_planning_scene is None
        self._active_payload: dict[str, Any] | None = None
        self._payload_generation = 0

    def status(self) -> BackendStatus:
        """复用机械臂连接与空闲状态作为组合动作的派发前门禁。"""

        return self.arm_backend.status()

    def execute(self, command: RobotCommand) -> CommandResult:
        """执行一个完整 AccessMotionBlock，并保留部分执行后的不确定性。

        参数：命令的每个段必须显式携带 ``phase_kind``。返回：组合终态。
        异常：首次物理作用前的结构错误为拒绝；之后任何异常均为派发不明。
        """

        phases = self._prevalidate(command)
        completed: list[Mapping[str, str]] = []
        payload_transition: Mapping[str, Any] | None = None
        physical_effect = False
        try:
            for segment, phase_kind in phases:
                if phase_kind is WorkCellPhaseKind.ARM_MOVE:
                    child_id = f"{command.command_id}:arm:{segment.segment_id}"
                    with self._active_lock:
                        self._active_arm_command_id = child_id
                    try:
                        try:
                            result = self.arm_backend.execute(
                                replace(
                                    command,
                                    command_id=child_id,
                                    segments=(segment,),
                                )
                            )
                        except DispatchUnknownError as exc:
                            physical_effect = True
                            output: dict[str, Any] = {
                                "phases": completed,
                                "unknown_phase": segment.segment_id,
                            }
                            if payload_transition is not None:
                                output["payload_attachment"] = dict(
                                    payload_transition
                                )
                            unknown = CommandResult(
                                command.command_id,
                                CommandState.EXECUTION_UNKNOWN,
                                "AccessMotionBlock 在 "
                                f"{segment.segment_id} 派发结果不明: {exc}",
                                output,
                            )
                            self._results[command.command_id] = unknown
                            return unknown
                    finally:
                        with self._active_lock:
                            self._active_arm_command_id = None
                    physical_effect = True
                elif phase_kind is WorkCellPhaseKind.END_EFFECTOR:
                    result = self._execute_end_effector(command, segment.target_ref)
                    physical_effect = True
                elif phase_kind is WorkCellPhaseKind.OBSERVE:
                    result = self._observe_payload(command, segment.parameters)
                else:
                    raise CommandRejectedError(
                        "standalone Arm 的 AccessMotionBlock 不得包含导轨阶段"
                    )
                completed.append(
                    {
                        "segment_id": segment.segment_id,
                        "target_ref": segment.target_ref,
                        "state": result.state.value,
                    }
                )
                if result.state is not CommandState.SUCCEEDED:
                    return self._non_success(
                        command,
                        result,
                        completed=completed,
                        physical_effect=physical_effect,
                        payload_transition=payload_transition,
                    )
                if phase_kind is WorkCellPhaseKind.OBSERVE:
                    payload_observation = result.output.get("payload_observation")
                    if not isinstance(payload_observation, Mapping):
                        raise DispatchUnknownError("负载观测结果缺少结构化见证")
                    payload_transition = self._update_payload_attachment(
                        command,
                        payload_observation,
                    )
        except CommandRejectedError:
            if not physical_effect:
                raise
            raise DispatchUnknownError(
                "AccessMotionBlock 已产生物理作用，后续阶段被拒绝"
            )
        except DispatchUnknownError:
            raise
        except Exception as exc:
            if not physical_effect:
                raise CommandRejectedError(
                    f"AccessMotionBlock 派发前失败: {exc}"
                ) from exc
            raise DispatchUnknownError(
                f"AccessMotionBlock 部分执行后结果不明: {exc}"
            ) from exc
        if (
            payload_transition is not None
            and payload_transition.get("state") == "detached"
            and self.payload_planning_scene is not None
        ):
            try:
                clearance_receipt = self._validated_clearance_receipt(
                    self.payload_planning_scene.finalize_detached_payload(
                        command_id=command.command_id,
                        payload_instance_ref=self._payload_instance_ref(command),
                        payload_profile_ref=command.payload_profile,
                        attachment_generation=int(
                            payload_transition["attachment_generation"]
                        ),
                    ),
                    command=command,
                    attachment_generation=int(
                        payload_transition["attachment_generation"]
                    ),
                    parent_link=str(payload_transition["parent_link"]),
                )
            # 任意 Adapter 异常都发生在机械臂已经撤离之后；此处必须统一提升为
            # EXECUTION_UNKNOWN，不能让调用栈类型决定是否错误地清除 Fence。
            except Exception as exc:  # noqa: BLE001
                unknown_transition = dict(payload_transition)
                unknown = CommandResult(
                    command.command_id,
                    CommandState.EXECUTION_UNKNOWN,
                    "AccessMotionBlock 已完成机械臂撤离，但无法确认临时碰撞许可已撤销: "
                    f"{exc}",
                    {
                        "phases": completed,
                        "unknown_phase": "finalize-detach-clearance",
                        "payload_attachment": unknown_transition,
                    },
                )
                self._results[command.command_id] = unknown
                return unknown
            payload_transition = {
                **dict(payload_transition),
                "clearance_receipt": clearance_receipt,
            }
        output: dict[str, Any] = {"phases": completed}
        if payload_transition is not None:
            output["payload_attachment"] = dict(payload_transition)
        result = CommandResult(
            command.command_id,
            CommandState.SUCCEEDED,
            "AccessMotionBlock 的机械臂、夹爪与负载观测全部完成",
            output,
        )
        self._results[command.command_id] = result
        return result

    def reconcile(self, command_id: str) -> CommandResult:
        """返回已知组合终态；缺失完整证据时保持 execution_unknown。"""

        return self._results.get(
            command_id,
            CommandResult(
                command_id,
                CommandState.EXECUTION_UNKNOWN,
                "组合动作缺少完整机械臂与夹爪完成见证",
            ),
        )

    def validate_before_dispatch(self, command: RobotCommand) -> None:
        """在 WorkCell 移动导轨前完成组合结构与持料身份校验。"""

        self._prevalidate(command)

    def request_stop(self, command_id: str, reason: str) -> CommandResult:
        """请求机械臂受控停止；夹爪没有通用停止证明，因此不解除 Fence。"""

        with self._active_lock:
            active_arm_command_id = self._active_arm_command_id
        self.arm_backend.request_stop(active_arm_command_id or command_id, reason)
        result = CommandResult(
            command_id,
            CommandState.EXECUTION_UNKNOWN,
            "已请求组合动作受控停止，等待物理结算",
        )
        self._results[command_id] = result
        return result

    def prepare_unknown_settlement(self, command_id: str) -> Mapping[str, Any]:
        """用新鲜工具/夹爪观测和 MoveIt 场景准备 UNKNOWN 物理结算。

        本方法不执行运动。只有场景端口确认“持久记录仍 attached、真实 mesh
        已消失”且夹爪明确空载时，才返回可向上投影的 detached 转移。
        """

        command_ref = str(command_id).strip()
        if not command_ref:
            raise CommandRejectedError("UNKNOWN 结算 command_id 不能为空")
        scene = self.payload_planning_scene
        if scene is None:
            return {}
        tool = self.tool_changer.observe()
        context = self.tool_changer.active_tool_context
        tool_age = time.time() - tool.observed_at
        if not (
            tool.state is ObservationState.KNOWN
            and 0.0 <= tool_age <= tool.max_age_s
            and tool.locked is True
            and tool.tool_ref == self.expected_tool_context.context_id
            and tool.attachment_generation
            == self.expected_tool_context.attachment_generation
            and context.digest == self.expected_tool_context.digest
        ):
            raise CommandRejectedError("UNKNOWN 结算缺少新鲜、锁紧且同摘要的工具见证")
        payload = self.end_effector.observe()
        payload_age = time.time() - payload.observed_at
        if not (
            payload.state is ObservationState.KNOWN
            and 0.0 <= payload_age <= payload.max_age_s
            and isinstance(payload.holding_payload, bool)
        ):
            raise CommandRejectedError("UNKNOWN 结算缺少新鲜夹爪负载见证")
        receipt = scene.settle_unknown_payload_state(
            command_id=command_ref,
            holding_payload=payload.holding_payload,
        )
        if receipt is None:
            snapshot = list(scene.snapshot())
            if len(snapshot) > 1:
                raise CommandRejectedError("UNKNOWN 结算后存在多个持料碰撞体")
            self._active_payload = dict(snapshot[0]) if snapshot else None
            self._payload_generation = int(
                (self._active_payload or {}).get("attachment_generation", 0)
            )
            self._payload_scene_state_ready = True
            return {}
        result = dict(receipt)
        parent_link = str(
            self.expected_tool_context.planning_scene.get("parent_link") or ""
        ).strip()
        required_text = (
            "payload_instance_ref",
            "payload_profile_ref",
            "payload_profile_digest",
        )
        if (
            result.get("state") != "detached"
            or result.get("parent_link") != parent_link
            or any(not str(result.get(field) or "").strip() for field in required_text)
            or not isinstance(result.get("attachment_generation"), int)
            or int(result["attachment_generation"]) < 1
        ):
            raise CommandRejectedError("UNKNOWN detach 场景收据身份、代次或工具锚点无效")
        transition = {
            "state": "detached",
            "evidence": "observed",
            "payload_profile_ref": result["payload_profile_ref"],
            "payload_instance_ref": result["payload_instance_ref"],
            "attachment_generation": result["attachment_generation"],
            "parent_link": parent_link,
            "observed_at": payload.observed_at,
            "stale_after_s": payload.max_age_s,
            "scene_receipt": result,
        }
        self._active_payload = None
        self._payload_generation = int(result["attachment_generation"])
        self._payload_scene_state_ready = True
        return {"payload_attachment": transition}

    def end_effector_observation(self):
        """保留原机械臂后端的末端观测兼容表面。"""

        return self.arm_backend.end_effector_observation()

    def _prevalidate(
        self,
        command: RobotCommand,
    ) -> tuple[tuple[MotionSegment, WorkCellPhaseKind], ...]:
        """首次物理作用前校验所有阶段形态与夹爪初态。"""

        self._ensure_payload_scene_state()
        result: list[tuple[MotionSegment, WorkCellPhaseKind]] = []
        seen_end_effector = 0
        seen_observation = 0
        for segment in command.segments:
            raw_kind = segment.parameters.get("phase_kind")
            try:
                kind = WorkCellPhaseKind(str(raw_kind))
            except ValueError as exc:
                raise CommandRejectedError(
                    f"AccessMotionBlock 阶段类型无效: {raw_kind}"
                ) from exc
            if kind is WorkCellPhaseKind.RAIL_MOVE:
                raise CommandRejectedError("standalone AccessMotionBlock 禁止导轨阶段")
            if kind is WorkCellPhaseKind.END_EFFECTOR:
                seen_end_effector += 1
                expected = (
                    "end_effector.grip"
                    if command.action.value == "pick"
                    else "end_effector.release"
                )
                if segment.target_ref != expected:
                    raise CommandRejectedError("夹爪阶段与通用动作不匹配")
            if kind is WorkCellPhaseKind.OBSERVE:
                seen_observation += 1
            result.append((segment, kind))
        phase_kinds = tuple(kind for _, kind in result)
        is_pour = command.action.value == "pour"
        if is_pour:
            if (
                len(result) < 3
                or any(kind is not WorkCellPhaseKind.ARM_MOVE for kind in phase_kinds)
                or result[0][0].target_ref != result[-1][0].target_ref
            ):
                raise CommandRejectedError(
                    "pour 必须是保持持料的 Arm ready→tilt→ready 闭合运动块"
                )
        else:
            if seen_end_effector != 1 or seen_observation != 1:
                raise CommandRejectedError(
                    "pick/place 必须各含一个夹爪阶段和负载观测阶段"
                )
            end_effector_index = phase_kinds.index(WorkCellPhaseKind.END_EFFECTOR)
            observation_index = phase_kinds.index(WorkCellPhaseKind.OBSERVE)
            if (
                end_effector_index < 1
                or observation_index != end_effector_index + 1
                or observation_index >= len(phase_kinds) - 1
                or any(
                    kind is not WorkCellPhaseKind.ARM_MOVE
                    for kind in phase_kinds[:end_effector_index]
                )
                or any(
                    kind is not WorkCellPhaseKind.ARM_MOVE
                    for kind in phase_kinds[observation_index + 1 :]
                )
            ):
                raise CommandRejectedError(
                    "pick/place 必须严格按 Arm 接近→夹爪→负载观测→Arm 撤离执行"
                )
        observation = self.end_effector.observe()
        attachment = self.tool_changer.observe()
        active_context = self.tool_changer.active_tool_context
        tool_known = (
            attachment.state is ObservationState.KNOWN
            and 0.0
            <= time.time() - attachment.observed_at
            <= attachment.max_age_s
            and attachment.locked is True
            and attachment.tool_ref == self.expected_tool_context.context_id
            and attachment.attachment_generation
            == self.expected_tool_context.attachment_generation
            and active_context.digest == self.expected_tool_context.digest
        )
        if not tool_known:
            raise CommandRejectedError(
                "快换工具身份、锁紧、附着代次或 ToolContext 摘要不匹配"
            )
        if (
            observation.state is not ObservationState.KNOWN
            or not 0.0
            <= time.time() - observation.observed_at
            <= observation.max_age_s
            or observation.holding_payload is None
        ):
            raise CommandRejectedError("夹爪初始观测未知或过期")
        expected_holding = command.action.value in {"place", "pour"}
        if observation.holding_payload is not expected_holding:
            raise CommandRejectedError("夹爪初始负载状态与动作不匹配")
        if self.require_payload_collision and self.payload_planning_scene is None:
            raise CommandRejectedError("MoveIt pick/place 未注入 PayloadPlanningScenePort")
        if self.require_payload_collision and not str(
            self.expected_tool_context.planning_scene.get("parent_link") or ""
        ).strip():
            raise CommandRejectedError(
                "MoveIt pick/place 的 ToolContext 缺少完整限定 parent_link"
            )
        if command.action.value == "pick" and self._active_payload is not None:
            raise CommandRejectedError("pick 前 PlanningScene 已存在持料碰撞体")
        payload_instance_ref = self._payload_instance_ref(command)
        if command.action.value in {"place", "pour"}:
            if self._active_payload is None:
                raise CommandRejectedError(
                    f"{command.action.value} 前 PlanningScene 不存在持料碰撞体"
                )
            if self._active_payload.get("payload_profile_ref") != command.payload_profile:
                raise CommandRejectedError(
                    f"{command.action.value} 负载 Profile 与当前持料碰撞体不一致"
                )
            if (
                self._active_payload.get("payload_instance_ref")
                != payload_instance_ref
            ):
                raise CommandRejectedError(
                    f"{command.action.value} 负载实例与当前持料碰撞体不一致"
                )
        return tuple(result)

    def _ensure_payload_scene_state(self) -> None:
        """在第一条命令派发前恢复同一夹爪的真实持料状态。"""

        if self._payload_scene_state_ready:
            return
        with self._active_lock:
            if self._payload_scene_state_ready:
                return
            scene = self.payload_planning_scene
            if scene is None:
                self._payload_scene_state_ready = True
                return
            scene_snapshot = list(scene.snapshot())
            if len(scene_snapshot) > 1:
                raise CommandRejectedError(
                    "单夹爪 PlanningScene 同时存在多个持料碰撞体"
                )
            self._active_payload = (
                dict(scene_snapshot[0]) if scene_snapshot else None
            )
            self._payload_generation = int(
                (self._active_payload or {}).get("attachment_generation", 0)
            )
            self._payload_scene_state_ready = True

    def _update_payload_attachment(
        self,
        command: RobotCommand,
        observation: Mapping[str, Any],
    ) -> Mapping[str, Any]:
        """在夹爪确定观测后同步 PlanningScene，并生成后端无关状态转移。"""

        parent_link = str(
            self.expected_tool_context.planning_scene.get("parent_link") or ""
        ).strip()
        payload_instance_ref = self._payload_instance_ref(command)
        if command.action.value == "pick":
            self._payload_generation += 1
            generation = self._payload_generation
            scene_receipt: Mapping[str, Any] = {}
            if self.payload_planning_scene is not None:
                scene_receipt = self.payload_planning_scene.attach_payload(
                    command_id=command.command_id,
                    payload_instance_ref=payload_instance_ref,
                    payload_profile_ref=command.payload_profile,
                    attachment_generation=generation,
                )
                scene_receipt = self._validated_scene_receipt(
                    scene_receipt,
                    command=command,
                    attachment_generation=generation,
                    parent_link=parent_link,
                    state="attached",
                )
            transition = {
                "state": "attached",
                "evidence": "observed",
                "payload_profile_ref": command.payload_profile,
                "payload_instance_ref": payload_instance_ref,
                "attachment_generation": generation,
                "parent_link": parent_link,
                "observed_at": float(observation["observed_at"]),
                "stale_after_s": float(observation["max_age_s"]),
                "scene_receipt": dict(scene_receipt),
            }
            self._active_payload = dict(transition)
            return transition
        active = self._active_payload
        if active is None:
            raise DispatchUnknownError("place 释放后缺少当前持料上下文")
        generation = int(active["attachment_generation"])
        scene_receipt = {}
        if self.payload_planning_scene is not None:
            scene_receipt = self._validated_scene_receipt(
                self.payload_planning_scene.detach_payload(
                    command_id=command.command_id,
                    payload_instance_ref=payload_instance_ref,
                    payload_profile_ref=command.payload_profile,
                    attachment_generation=generation,
                ),
                command=command,
                attachment_generation=generation,
                parent_link=parent_link,
                state="detached",
            )
        transition = {
            "state": "detached",
            "evidence": "observed",
            "payload_profile_ref": command.payload_profile,
            "payload_instance_ref": payload_instance_ref,
            "attachment_generation": generation,
            "parent_link": parent_link,
            "observed_at": float(observation["observed_at"]),
            "stale_after_s": float(observation["max_age_s"]),
            "scene_receipt": scene_receipt,
        }
        self._active_payload = None
        return transition

    @staticmethod
    def _validated_scene_receipt(
        receipt: Mapping[str, Any],
        *,
        command: RobotCommand,
        attachment_generation: int,
        parent_link: str,
        state: str,
    ) -> dict[str, Any]:
        """拒绝与统一命令身份、代次或工具锚点不一致的场景回读。"""

        result = dict(receipt)
        expected = {
            "payload_profile_ref": command.payload_profile,
            "payload_instance_ref": AccessMotionBackend._payload_instance_ref(
                command
            ),
            "attachment_generation": attachment_generation,
            "parent_link": parent_link,
            "state": state,
        }
        mismatches = [
            key for key, value in expected.items() if result.get(key) != value
        ]
        if mismatches:
            raise DispatchUnknownError(
                "PayloadPlanningScene 回读与命令不一致: "
                + ", ".join(mismatches)
            )
        return result

    @staticmethod
    def _validated_clearance_receipt(
        receipt: Mapping[str, Any],
        *,
        command: RobotCommand,
        attachment_generation: int,
        parent_link: str,
    ) -> dict[str, Any]:
        """校验撤离后的世界物料与临时碰撞许可收据。"""

        result = dict(receipt)
        expected = {
            "command_id": command.command_id,
            "payload_profile_ref": command.payload_profile,
            "payload_instance_ref": AccessMotionBackend._payload_instance_ref(
                command
            ),
            "attachment_generation": attachment_generation,
            "parent_link": parent_link,
            "state": "detached_clearance_confirmed",
            "world_body_present": True,
        }
        mismatches = [
            key for key, value in expected.items() if result.get(key) != value
        ]
        if mismatches or not isinstance(
            result.get("contact_allowance_revoked"), bool
        ):
            details = mismatches or ["contact_allowance_revoked"]
            raise DispatchUnknownError(
                "PayloadPlanningScene 撤离回读与命令不一致: "
                + ", ".join(details)
            )
        return result

    @staticmethod
    def _payload_instance_ref(command: RobotCommand) -> str:
        """读取部署层解析的运行期负载身份；禁止用 Profile 代替实例。"""

        value = str(command.metadata.get("payload_instance_ref") or "").strip()
        if not value:
            raise CommandRejectedError(
                "pick/place 命令缺少运行期 payload_instance_ref"
            )
        return value

    def _execute_end_effector(
        self,
        command: RobotCommand,
        target_ref: str,
    ) -> CommandResult:
        """用子命令身份执行抓取或释放。"""

        child_id = f"{command.command_id}:end-effector:{target_ref}"
        if target_ref == "end_effector.grip":
            return self.end_effector.grip(
                child_id,
                payload_profile=command.payload_profile,
            )
        if target_ref == "end_effector.release":
            return self.end_effector.release(child_id)
        raise CommandRejectedError(f"未知夹爪动作: {target_ref}")

    def _observe_payload(
        self,
        command: RobotCommand,
        parameters: Mapping[str, object],
    ) -> CommandResult:
        """验证夹爪负载观测与计划声明的出站状态一致。"""

        expected_state = str(parameters.get("payload_state", ""))
        if expected_state not in {"loaded", "empty"}:
            raise CommandRejectedError("负载观测阶段缺少 loaded/empty 目标")
        observation = self.end_effector.observe()
        known = (
            observation.state is ObservationState.KNOWN
            and 0.0
            <= time.time() - observation.observed_at
            <= observation.max_age_s
            and observation.holding_payload is (expected_state == "loaded")
        )
        return CommandResult(
            command.command_id,
            CommandState.SUCCEEDED if known else CommandState.EXECUTION_UNKNOWN,
            "夹爪负载观测已确认" if known else "夹爪负载观测未知、过期或不匹配",
            {
                "payload_observation": {
                    "observed_at": observation.observed_at,
                    "max_age_s": observation.max_age_s,
                    "source": observation.source,
                }
            },
        )

    def _non_success(
        self,
        command: RobotCommand,
        result: CommandResult,
        *,
        completed: list[Mapping[str, str]],
        physical_effect: bool,
        payload_transition: Mapping[str, Any] | None = None,
    ) -> CommandResult:
        """部分执行后不把失败折叠为可重试普通失败。"""

        state = CommandState.EXECUTION_UNKNOWN if physical_effect else result.state
        output: dict[str, Any] = {"phases": completed}
        if payload_transition is not None:
            output["payload_attachment"] = dict(payload_transition)
        combined = CommandResult(
            command.command_id,
            state,
            f"AccessMotionBlock 在 {completed[-1]['segment_id']} 停止: {result.message}",
            output,
        )
        self._results[command.command_id] = combined
        return combined


__all__ = ["AccessMotionBackend", "PayloadPlanningScenePort"]
