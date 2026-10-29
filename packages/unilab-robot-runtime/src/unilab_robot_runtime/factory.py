"""依据硬件配置（HardwareProfile）选择并装配通用机器人运行时。"""

from __future__ import annotations

import importlib
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import yaml
from unilab_rail_mounted_arm import (
    ConfiguredInterlockProvider,
    InterlockBinding,
    RailMountedArmCoordinator,
    SimulationInterlockProvider,
    SimulationRailMountedArmRuntime,
)
from unilab_robot_contracts import (
    BackendKind,
    DeploymentMode,
    HardwareProfile,
    InstallationCalibration,
    ObservationState,
    RigidTransform,
    RobotPointSetResolver,
    SafetyInterlockObservation,
    SQLiteCommandJournal,
    ToolContext,
    ToolDefinition,
)

from .access_motion_backend import AccessMotionBackend
from .binding import RuntimeBinding, bind_runtime


class ModuleReference(Protocol):
    """运行时选择模块所需的最小 exact 引用。"""

    distribution: str
    version: str
    endpoint_ids: frozenset[str]
    python_package: str


class RuntimeManifest(Protocol):
    """领域部署清单向 Robotics 提供的最小只读接口。"""

    deployment_id: str
    profile: HardwareProfile
    arm: ModuleReference
    rail: ModuleReference | None

    def asset_path(self, name: str) -> Path:
        """返回已完成 exact digest 校验的部署资产路径。"""


@dataclass(frozen=True)
class RuntimeRequirements:
    """运行时要求领域组合根注入的外部能力，不泄漏后端判断。"""

    variable_port: bool = False
    moveit_client: bool = False
    robot_sdk_port: bool = False
    end_effector_port: bool = False
    tool_changer_port: bool = False


@dataclass(frozen=True)
class RuntimeDependencies:
    """领域/OS 对通用运行时的依赖注入集合。"""

    runtime_root: str | Path
    variable_port: Any = None
    moveit_client: Any = None
    qualified_joint_names: tuple[str, ...] | None = None
    robot_sdk_port: Any = None
    end_effector_port: Any = None
    tool_changer_port: Any = None
    safety_observation: Callable[[], SafetyInterlockObservation] | None = None


def runtime_requirements(manifest: RuntimeManifest) -> RuntimeRequirements:
    """由 Robotics 内部解释硬件配置并返回依赖要求。

    参数：已校验部署清单。返回：互斥的外部能力要求。
    异常：不支持的后端或组合形态时关闭失败。
    """

    _validate_module(manifest.arm, kind="arm")
    if manifest.rail is not None:
        _validate_module(manifest.rail, kind="rail")
    backend = manifest.profile.backend
    if backend is BackendKind.PLC:
        return RuntimeRequirements(variable_port=True)
    if manifest.rail is not None:
        raise ValueError("非 PLC 的 RailMountedArm 尚无受支持的异构 Rail Adapter")
    if backend is BackendKind.MOVEIT:
        return RuntimeRequirements(
            moveit_client=True,
            end_effector_port=manifest.profile.mode is not DeploymentMode.SIMULATION,
            tool_changer_port=manifest.profile.mode is not DeploymentMode.SIMULATION,
        )
    if backend is BackendKind.TCP_SDK:
        return RuntimeRequirements(
            robot_sdk_port=True,
            end_effector_port=manifest.profile.mode is not DeploymentMode.SIMULATION,
            tool_changer_port=manifest.profile.mode is not DeploymentMode.SIMULATION,
        )
    raise ValueError(f"不支持的 RobotExecutionBackend: {backend}")


def create_runtime(
    manifest: RuntimeManifest,
    dependencies: RuntimeDependencies,
) -> RuntimeBinding:
    """按硬件配置原子装配 standalone Arm 或 RailMountedArm WorkCell。

    参数：exact 清单与由领域/OS 注入的能力。返回：唯一公共运行时绑定。
    异常：依赖缺失、资产错误、安全模式不合规或端点重复时关闭失败。
    """

    requirements = runtime_requirements(manifest)
    runtime_root = Path(dependencies.runtime_root)
    if requirements.variable_port:
        if dependencies.variable_port is None:
            raise ValueError("PLC runtime 必须注入变量端口")
        return _create_plc_runtime(manifest, dependencies.variable_port, runtime_root)
    if requirements.end_effector_port and dependencies.end_effector_port is None:
        raise ValueError("非仿真 PointSet runtime 必须注入独立 EndEffectorPort")
    if requirements.tool_changer_port and dependencies.tool_changer_port is None:
        raise ValueError("非仿真 PointSet runtime 必须注入独立 ToolChangerPort")
    if requirements.moveit_client:
        if dependencies.moveit_client is None or not dependencies.qualified_joint_names:
            raise ValueError("MoveIt runtime 必须注入 client 与完整关节名")
        return _create_moveit_runtime(manifest, dependencies, runtime_root)
    if dependencies.robot_sdk_port is None:
        raise ValueError("TCP/SDK runtime 必须注入受限 RobotSDKPort")
    return _create_tcp_sdk_runtime(manifest, dependencies, runtime_root)


def arm_model_descriptor(manifest: RuntimeManifest) -> Any:
    """返回 exact Arm distribution 拥有的型号描述，供 OS 创建 MoveIt client。"""

    return _module_impl(manifest.arm, kind="arm").MODEL_DESCRIPTOR


def _create_plc_runtime(
    manifest: RuntimeManifest,
    port: Any,
    runtime_root: Path,
) -> RuntimeBinding:
    """装配 PLC standalone Arm 或 RailMountedArm WorkCell。"""

    program_data = _load_yaml(
        manifest.asset_path("program_set"),
        "unilab.plc-program-set/v1",
    )
    adapter_data = _load_yaml(
        manifest.asset_path("plc_adapter"),
        "unilab.robot-plc-adapter/v1",
    )
    arm_impl = _module_impl(manifest.arm, kind="arm")
    arm_backend = arm_impl.create_plc_backend(
        port=port,
        endpoint_ids=manifest.arm.endpoint_ids,
        program_data=program_data,
        adapter_data=adapter_data,
    )
    package_local_simulation = (
        manifest.rail is not None and manifest.profile.mode is DeploymentMode.SIMULATION
    )
    simulation_interlock: SimulationInterlockProvider | None = None
    if package_local_simulation:
        simulation_interlock = SimulationInterlockProvider()
        safety_observation = simulation_interlock.read
    else:
        interlock_data = _load_yaml(
            manifest.asset_path("interlock_profile"),
            "unilab.robot-interlock/v1",
        )
        safety_observation = ConfiguredInterlockProvider(
            port=port,
            binding=_interlock_binding(interlock_data),
        ).read
    arm_journal_name = (
        "public-commands.sqlite3" if manifest.rail is None else "arm-private.sqlite3"
    )
    arm = arm_impl.create_arm_module(
        backend=arm_backend,
        profile=manifest.profile,
        journal=SQLiteCommandJournal(runtime_root / arm_journal_name),
        safety_observation=safety_observation,
    )
    if manifest.rail is None:
        return bind_runtime(
            arm,
            arm.endpoint_ids,
            owner_id=manifest.deployment_id,
            rail_mounted=False,
        )
    rail_impl = _module_impl(manifest.rail, kind="rail")
    rail_data = _load_yaml(
        manifest.asset_path("rail_target_set"),
        "unilab.rail-target-set/v1",
    )
    if package_local_simulation:
        if simulation_interlock is None:
            raise RuntimeError("仿真互锁未完成装配")
        rail = rail_impl.create_simulation_module(
            endpoint_ids=manifest.rail.endpoint_ids,
            target_data=rail_data,
            on_settled=simulation_interlock.rail_settled,
        )
    else:
        rail = rail_impl.create_plc_module(
            port=port,
            endpoint_ids=manifest.rail.endpoint_ids,
            target_data=rail_data,
            adapter_data=adapter_data,
        )
    coordinator = RailMountedArmCoordinator(
        arm=arm,
        rail=rail,
        profile=manifest.profile,
        journal=SQLiteCommandJournal(runtime_root / "public-commands.sqlite3"),
        safety_observation=safety_observation,
    )
    runtime: Any = coordinator
    if package_local_simulation:
        if simulation_interlock is None:
            raise RuntimeError("仿真互锁未完成装配")
        runtime = SimulationRailMountedArmRuntime(
            coordinator=coordinator,
            interlock=simulation_interlock,
        )
    return bind_runtime(
        runtime,
        coordinator.endpoint_ids,
        owner_id=manifest.deployment_id,
        rail_mounted=True,
    )


def _create_moveit_runtime(
    manifest: RuntimeManifest,
    dependencies: RuntimeDependencies,
    runtime_root: Path,
) -> RuntimeBinding:
    """装配 headless MoveIt standalone Arm；RViz 不参与生命周期。"""

    safety_observation = dependencies.safety_observation
    if safety_observation is None:
        if manifest.profile.mode is not DeploymentMode.SIMULATION:
            raise ValueError("非仿真 MoveIt 必须注入真实安全观测")
        safety_observation = _standalone_simulation_safety
    arm_impl = _module_impl(manifest.arm, kind="arm")
    tool_context = _load_tool_context(manifest)
    resolver = load_point_set(manifest)
    targets = resolver.arm_targets
    backend = arm_impl.create_moveit_backend(
        moveit_client=dependencies.moveit_client,
        endpoint_ids=manifest.arm.endpoint_ids,
        targets=targets,
        qualified_joint_names=dependencies.qualified_joint_names,
    )
    end_effector, tool_changer = _manipulation_ports(
        manifest,
        dependencies,
        tool_context,
    )
    backend = AccessMotionBackend(
        arm_backend=backend,
        end_effector=end_effector,
        tool_changer=tool_changer,
        expected_tool_context=tool_context,
    )
    commissioning = arm_impl.create_moveit_commissioning_adapter(
        moveit_client=dependencies.moveit_client,
        targets=targets,
        qualified_joint_names=dependencies.qualified_joint_names,
        point_set_revision=resolver.revision,
        hardware_profile_digest=manifest.profile.digest,
        tool_context_digest=tool_context.digest,
        commissioning_velocity_limit=manifest.profile.commissioning_velocity_limit,
        commissioning_acceleration_limit=manifest.profile.commissioning_acceleration_limit,
        joint_completion_tolerance_si=(
            manifest.profile.commissioning_joint_completion_tolerance_si
        ),
    )
    binding = _create_standalone_arm(
        manifest,
        arm_impl,
        backend,
        safety_observation,
        runtime_root,
        commissioning_port=commissioning,
    )
    try:
        commissioning.activate_tool_context(tool_context)
    except Exception:
        binding.close()
        raise
    return binding


def _create_tcp_sdk_runtime(
    manifest: RuntimeManifest,
    dependencies: RuntimeDependencies,
    runtime_root: Path,
) -> RuntimeBinding:
    """装配 TCP/SDK standalone Arm，安全观测必须显式注入。"""

    if dependencies.safety_observation is None:
        raise ValueError("TCP/SDK runtime 必须注入真实或显式仿真安全观测")
    arm_impl = _module_impl(manifest.arm, kind="arm")
    resolver = load_point_set(manifest)
    tool_context = _load_tool_context(manifest)
    backend = arm_impl.create_tcp_sdk_backend(
        port=dependencies.robot_sdk_port,
        endpoint_ids=manifest.arm.endpoint_ids,
        targets=resolver.arm_targets,
    )
    end_effector, tool_changer = _manipulation_ports(
        manifest,
        dependencies,
        tool_context,
    )
    backend = AccessMotionBackend(
        arm_backend=backend,
        end_effector=end_effector,
        tool_changer=tool_changer,
        expected_tool_context=tool_context,
    )
    return _create_standalone_arm(
        manifest,
        arm_impl,
        backend,
        dependencies.safety_observation,
        runtime_root,
    )


def _create_standalone_arm(
    manifest: RuntimeManifest,
    arm_impl: Any,
    backend: Any,
    safety_observation: Callable[[], SafetyInterlockObservation],
    runtime_root: Path,
    commissioning_port: Any = None,
) -> RuntimeBinding:
    """复用相同模块和账本装配任意 standalone 执行后端。"""

    arm = arm_impl.create_arm_module(
        backend=backend,
        profile=manifest.profile,
        journal=SQLiteCommandJournal(runtime_root / "public-commands.sqlite3"),
        safety_observation=safety_observation,
    )
    return bind_runtime(
        arm,
        arm.endpoint_ids,
        owner_id=manifest.deployment_id,
        rail_mounted=False,
        commissioning_port=commissioning_port,
        deployment_mode=manifest.profile.mode,
    )


def _module_impl(ref: ModuleReference, *, kind: str) -> Any:
    """加载 exact 型号模块并在激活阶段验证 API、类型和版本。"""

    module = importlib.import_module(ref.python_package)
    if getattr(module, "MODULE_API_VERSION", None) != 1:
        raise ValueError(f"{ref.distribution} 不支持 Robot Module API v1")
    if str(getattr(module, "MODULE_KIND", "")) != kind:
        raise ValueError(f"{ref.distribution} 不是 {kind} 模块")
    if str(getattr(module, "__version__", "")) != ref.version:
        raise ValueError(f"{ref.distribution} 安装版本与 manifest 不一致")
    return module


def _load_yaml(path: Path, schema: str) -> Mapping[str, Any]:
    """读取 exact 资产，并拒绝 schema 不匹配。"""

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, Mapping) or data.get("schema") != schema:
        raise ValueError(f"资产 {path.name} schema 必须为 {schema}")
    return data


def _load_tool_context(manifest: RuntimeManifest) -> ToolContext:
    """加载活动工具、TCP 变换和附着代次。"""

    data = _load_yaml(manifest.asset_path("tool_context"), "unilab.tool-context/v1")
    transform = data.get("mount_to_tcp")
    if not isinstance(transform, Mapping):
        raise TypeError("ToolContext.mount_to_tcp 必须是对象")
    xyz = tuple(float(value) for value in transform.get("xyz_m", ()))
    orientation = tuple(float(value) for value in transform.get("orientation_xyzw", ()))
    if len(xyz) != 3 or len(orientation) != 4:
        raise ValueError("ToolContext 必须包含 xyz_m[3] 与 orientation_xyzw[4]")
    asset_ref = manifest.assets["tool_context"]
    return ToolContext(
        context_id=str(data["context_id"]),
        digest=str(asset_ref.digest),
        mount_to_tcp=RigidTransform(xyz, orientation),
        attachment_generation=int(data["attachment_generation"]),
        planning_scene=dict(data.get("planning_scene") or {}),
    )


def _manipulation_ports(
    manifest: RuntimeManifest,
    dependencies: RuntimeDependencies,
    tool_context: ToolContext,
) -> tuple[Any, Any]:
    """选择真实夹爪/快换，或创建仅仿真可用的包内组合端口。"""

    if (
        dependencies.end_effector_port is not None
        and dependencies.tool_changer_port is not None
    ):
        return dependencies.end_effector_port, dependencies.tool_changer_port
    if (dependencies.end_effector_port is None) != (
        dependencies.tool_changer_port is None
    ):
        raise ValueError("EndEffectorPort 与 ToolChangerPort 必须成对注入")
    if manifest.profile.mode is not DeploymentMode.SIMULATION:
        raise ValueError("当前 PointSet runtime 缺少 EndEffectorPort")
    from unilab_end_effector_sim import SimulatedGripper, SimulatedToolChanger

    tool_ref = tool_context.context_id
    collision_ref = manifest.assets.get("collision_environment")
    changer = SimulatedToolChanger(
        tools={
            tool_ref: ToolDefinition(
                tool_ref=tool_ref,
                model_digest=tool_context.digest,
                mount_to_tcp=tool_context.mount_to_tcp,
                collision_asset_ref=(
                    "simulation:unmodeled"
                    if collision_ref is None
                    else f"sha256:{collision_ref.digest}"
                ),
            )
        },
        qualified_contexts={tool_ref: tool_context},
    )
    attached = changer.change_tool(
        f"activation:{manifest.deployment_id}:tool",
        tool_ref=tool_ref,
    )
    if not attached.success:
        raise ValueError(f"仿真快换初始化失败: {attached.message}")
    return SimulatedGripper(tool_changer=changer), changer


def load_point_set(
    manifest: RuntimeManifest,
) -> RobotPointSetResolver:
    """加载唯一 PointSet v3 及其精确安装标定。

    参数：部署清单、型号描述和活动工具。返回：完整复合点位解析器。
    异常：资产摘要、Schema、型号、工具或坐标标定不一致时关闭失败。
    """

    arm_model = _module_impl(manifest.arm, kind="arm").MODEL_DESCRIPTOR
    tool_context = _load_tool_context(manifest)
    point_path = manifest.asset_path("point_set")
    point_data = _load_yaml(point_path, "unilab.robot-point-set/v3")
    point_asset = manifest.assets["point_set"]
    calibration = _load_installation_calibration(manifest)
    rail_model = None
    if manifest.rail is not None:
        rail_model = _module_impl(manifest.rail, kind="rail").MODEL_DESCRIPTOR
    return RobotPointSetResolver(
        point_data,
        arm_model=arm_model,
        tool_context=tool_context,
        calibration=calibration,
        rail_model=rail_model,
        source_digest=str(point_asset.digest),
    )


def _load_installation_calibration(
    manifest: RuntimeManifest,
) -> InstallationCalibration:
    """读取 device-local frame 到 arm base 的版本化安装标定。"""

    data = _load_yaml(
        manifest.asset_path("installation_calibration"),
        "unilab.installation-calibration/v1",
    )
    frames = data.get("frames")
    if not isinstance(frames, Mapping) or not frames:
        raise ValueError("InstallationCalibration.frames 必须是非空对象")
    transforms: dict[str, RigidTransform] = {}
    for frame_ref, value in frames.items():
        if not isinstance(value, Mapping):
            raise TypeError(f"calibration frame {frame_ref} 必须是对象")
        xyz = tuple(float(item) for item in value.get("xyz_m", ()))
        quaternion = tuple(float(item) for item in value.get("orientation_xyzw", ()))
        if len(xyz) != 3 or len(quaternion) != 4:
            raise ValueError(
                f"calibration frame {frame_ref} 必须包含 xyz_m[3] 与 orientation_xyzw[4]"
            )
        transforms[str(frame_ref)] = RigidTransform(xyz, quaternion)
    asset_ref = manifest.assets["installation_calibration"]
    return InstallationCalibration(
        str(data["revision"]),
        str(asset_ref.digest),
        transforms,
    )


def _interlock_binding(data: Mapping[str, Any]) -> InterlockBinding:
    """把部署安全配置解析为通用硬件互锁绑定。"""

    return InterlockBinding(
        granted_variable=str(data["granted_variable"]),
        rail_permitted_variable=str(data["rail_permitted_variable"]),
        arm_permitted_variable=str(data["arm_permitted_variable"]),
        concurrent_blocked_variable=str(data["concurrent_blocked_variable"]),
        hardware_enforced=_required_bool(
            data.get("hardware_enforced"), "interlock.hardware_enforced"
        ),
        source=str(data["source"]),
        max_age_s=float(data.get("max_age_s", 0.5)),
    )


def _required_bool(value: Any, field: str) -> bool:
    """配置布尔值必须是真实 bool，禁止把字符串 ``false`` 解释为 True。"""

    if not isinstance(value, bool):
        raise TypeError(f"{field} 必须是 bool")
    return value


def _standalone_simulation_safety() -> SafetyInterlockObservation:
    """为 standalone 仿真提供进程内许可；生产模式不可调用。"""

    return SafetyInterlockObservation(
        ObservationState.KNOWN,
        time.time(),
        1.0,
        "simulation-only:standalone",
        True,
        False,
        False,
        True,
        True,
    )


def _validate_module(ref: ModuleReference, *, kind: str) -> None:
    """在返回外部依赖要求前先验证 exact 模块，避免领域层猜后端。"""

    _module_impl(ref, kind=kind)
