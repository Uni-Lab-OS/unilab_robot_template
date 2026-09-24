"""MoveIt 卡片 commissioning 资产 scaffold（PointSet v3 + moveit_commissioning）。"""

from __future__ import annotations

import hashlib
import re
from pathlib import Path
from typing import Any


def _boot_id_prefix(domain_pkg: str) -> str:
    return domain_pkg.replace("_", "")


def point_set_path(domain: Path, *, basename: str) -> Path:
    return (
        domain
        / "deployment"
        / "robot_cell"
        / "assets"
        / "point_sets"
        / f"{basename}.v3.yaml"
    )


def moveit_commissioning_path(domain: Path, *, domain_pkg: str, device_id: str) -> Path:
    return domain / domain_pkg / "devices" / device_id / "moveit_commissioning.py"


def point_set_v3_path(domain: Path, *, domain_pkg: str) -> Path:
    return domain / domain_pkg / "robot_cell" / "point_set_v3.py"


def rail_simulation_path(domain: Path, *, domain_pkg: str) -> Path:
    return domain / domain_pkg / "devices" / "rail_simulation.py"


def assess_moveit_commissioning(
    domain: Path,
    *,
    domain_pkg: str,
    device_id: str,
    rail_id: str,
    arm_card_path: Path | None,
) -> dict[str, bool | list[str]]:
    missing: list[str] = []
    basename = f"{domain_pkg}-rail-arm"
    for path in (
        point_set_path(domain, basename=basename),
        moveit_commissioning_path(domain, domain_pkg=domain_pkg, device_id=device_id),
        point_set_v3_path(domain, domain_pkg=domain_pkg),
        rail_simulation_path(domain, domain_pkg=domain_pkg),
    ):
        if not path.is_file():
            missing.append(str(path.relative_to(domain)).replace("\\", "/"))
    if arm_card_path is not None and arm_card_path.is_file():
        text = arm_card_path.read_text(encoding="utf-8")
        if "NotImplementedError" in text or "build_" not in text or "_arm_card_context()" not in text:
            missing.append(
                str(arm_card_path.relative_to(domain)).replace("\\", "/") + "#context"
            )
    rail_py = domain / domain_pkg / "devices" / f"{rail_id}.py"
    if rail_py.is_file() and "post_init" not in rail_py.read_text(encoding="utf-8"):
        missing.append(str(rail_py.relative_to(domain)).replace("\\", "/") + "#post_init")
    device_py = domain / domain_pkg / "devices" / device_id / "device.py"
    if device_py.is_file() and "_moveit_split_binding" not in device_py.read_text(encoding="utf-8"):
        missing.append(str(device_py.relative_to(domain)).replace("\\", "/") + "#post_init")
    return {"complete": not missing, "missing": missing}


def _render(text: str, mapping: dict[str, str]) -> str:
    rendered = text
    for key, value in mapping.items():
        rendered = rendered.replace(key, value)
    return rendered


def _point_set_yaml(*, revision: str, calibration_digest: str) -> str:
    return f"""schema: unilab.robot-point-set/v3
revision: {revision}
description: Greenfield 最小导轨+机械臂作者层点位（global home + demo 工位）
components:
  arm:
    model_ref: package://unilab_arm_cr5/models/model.yaml
    tool_context_ref: {revision.split('@', 1)[0]}-placeholder@1.0.0
  rail:
    model_ref: package://unilab_rail_linear/models/model.yaml
installation_calibration:
  revision: {revision.split('@', 1)[0]}-installation@1.0.0
  digest: {calibration_digest}
global:
  arm:
    home:
      type: joint_positions
      value: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    standby:
      type: joint_positions
      value: [0.0, -0.7, 1.2, 0.0, 0.9, 0.0]
    rail_transfer_safe:
      type: joint_positions
      value: [0.0, -0.7, 1.2, 0.0, 0.9, 0.0]
  rail:
    home:
      position_si: 0.0
demo-station:
  device_ref: deck.demo-station
  display_name: Demo 工位
  targets:
    ready:
      arm:
        type: joint_positions
        value: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      rail:
        position_si: 0.0
"""


def scaffold_moveit_commissioning(
    domain: Path,
    *,
    domain_pkg: str,
    device_id: str,
    rail_id: str,
    class_prefix: str,
    demo_dir: Path,
    apply: bool = False,
) -> dict[str, Any]:
    domain = domain.resolve()
    basename = f"{domain_pkg}-rail-arm"
    revision = f"{basename}@3.0.0"
    calibration_revision = f"{basename}-installation@1.0.0"
    calibration_digest = hashlib.sha256(calibration_revision.encode("utf-8")).hexdigest()
    boot_id = _boot_id_prefix(domain_pkg)
    tool_context_id = f"{basename}-placeholder@1.0.0"
    tool_context_digest = hashlib.sha256(tool_context_id.encode("utf-8")).hexdigest()
    mapping = {
        "{domain_pkg}": domain_pkg,
        "{device_id}": device_id,
        "{rail_id}": rail_id,
        "{class_prefix}": class_prefix,
        "{boot_id_prefix}": boot_id,
        "{point_set_basename}": basename,
        "{point_set_revision}": revision,
        "{tool_context_id}": tool_context_id,
        "{tool_context_digest}": tool_context_digest,
        "{calibration_revision}": calibration_revision,
        "{calibration_digest}": calibration_digest,
    }

    targets = {
        point_set_path(domain, basename=basename): _point_set_yaml(
            revision=revision,
            calibration_digest=calibration_digest,
        ),
        point_set_v3_path(domain, domain_pkg=domain_pkg): _render(
            (demo_dir / "point_set_v3.py.template").read_text(encoding="utf-8")
            if (demo_dir / "point_set_v3.py.template").is_file()
            else _default_point_set_v3_py(),
            mapping,
        ),
        moveit_commissioning_path(
            domain, domain_pkg=domain_pkg, device_id=device_id
        ): _render(
            (demo_dir / "moveit_commissioning.py.template").read_text(encoding="utf-8")
            if (demo_dir / "moveit_commissioning.py.template").is_file()
            else _default_moveit_commissioning_py(),
            mapping,
        ),
        rail_simulation_path(domain, domain_pkg=domain_pkg): _render(
            (demo_dir / "rail_simulation.py.template").read_text(encoding="utf-8")
            if (demo_dir / "rail_simulation.py.template").is_file()
            else _default_rail_simulation_py(),
            mapping,
        ),
        domain / domain_pkg / "robot_cell" / "__init__.py": (
            f'"""{domain_pkg} 机械臂单元（Robot Cell）资产与 PointSet 解析。"""\n'
        ),
    }

    arm_card_target = domain / domain_pkg / "devices" / device_id / f"{device_id}_arm_card.py"
    arm_template = demo_dir / "arm_card.py.template"
    if arm_template.is_file():
        arm_text = _render(arm_template.read_text(encoding="utf-8"), mapping)
        if apply and (
            not arm_card_target.is_file()
            or "NotImplementedError" in arm_card_target.read_text(encoding="utf-8")
        ):
            targets[arm_card_target] = arm_text

    planned: list[str] = []
    created: list[str] = []
    for path, content in targets.items():
        rel = str(path.relative_to(domain)).replace("\\", "/")
        if path.is_file():
            if path == arm_card_target and "NotImplementedError" in path.read_text(encoding="utf-8"):
                planned.append(rel)
                if apply:
                    path.write_text(content, encoding="utf-8", newline="\n")
                    created.append(rel)
            continue
        planned.append(rel)
        if apply:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content, encoding="utf-8", newline="\n")
            created.append(rel)

    return {
        "point_set_revision": revision,
        "planned_files": planned,
        "created_files": created,
        "changed": bool(planned),
    }


def patch_rail_post_init(source: str, *, domain_pkg: str) -> tuple[str, bool]:
    if "post_init" in source:
        return source, False
    if "rail_simulation" in source:
        return source, False
    marker = "from unilabos.registry.decorators import device\n"
    if marker not in source:
        return source, False
    updated = source.replace(
        marker,
        marker + "\nfrom typing import Any\n\nfrom unilabos.registry.decorators import not_action\n",
        1,
    )
    updated = updated.replace(
        "from unilabos.registry.decorators import device\n",
        "from unilabos.registry.decorators import device, not_action\n",
        1,
    )
    init_block = re.search(
        r"(def __init__\([\s\S]*?self\.config\.update\(kwargs\)\n)",
        updated,
    )
    if init_block is None:
        return source, False
    insert = (
        init_block.group(1)
        + "        self._simulation_driver = None\n\n"
        + "    @not_action\n"
        + "    def post_init(self, ros_node: object | None = None) -> None:\n"
        + f"        from {domain_pkg}.devices.rail_simulation import simulation_backends\n\n"
        + "        backend = str(self.config.get(\"standard_execution_backend\", \"\")).strip()\n"
        + "        if backend not in simulation_backends():\n"
        + "            return\n"
        + "        self._ensure_simulation_driver()\n"
        + "        if ros_node is not None:\n"
        + "            self._simulation_driver.attach_ros(ros_node)\n\n"
        + "    def _ensure_simulation_driver(self) -> None:\n"
        + f"        from {domain_pkg}.devices.rail_simulation import (\n"
        + "            RailSimulationDriver,\n"
        + "            register_rail_simulation_axis,\n"
        + "            simulation_backends,\n"
        + "        )\n\n"
        + "        backend = str(self.config.get(\"standard_execution_backend\", \"\")).strip()\n"
        + "        if backend not in simulation_backends():\n"
        + "            raise NotImplementedError\n"
        + "        if self._simulation_driver is None:\n"
        + "            self._simulation_driver = RailSimulationDriver(device_id=self.device_id)\n"
        + "        register_rail_simulation_axis(self.device_id, self._simulation_driver)\n"
    )
    updated = updated.replace(init_block.group(1), insert, 1)
    return updated, updated != source


def patch_device_moveit_post_init(
    source: str,
    *,
    domain_pkg: str,
    device_id: str,
) -> tuple[str, bool]:
    if "_moveit_split_binding" in source:
        return source, False
    if "post_init" in source:
        return source, False
    marker = "from unilabos.registry.decorators import device\n"
    if marker not in source:
        return source, False
    updated = source.replace(
        marker,
        "from typing import Any\n\nfrom unilabos.registry.decorators import device, not_action\n",
        1,
    )
    init_match = re.search(
        r"(def __init__\([\s\S]*?self\.config = dict\(config or \{\}\)\n)",
        updated,
    )
    if init_match is None:
        return source, False
    insert = (
        init_match.group(1)
        + "        self._moveit_split_binding: Any = None\n"
        + "        self._moveit_client: Any = None\n\n"
        + "    @not_action\n"
        + "    def post_init(self, ros_node: Any) -> None:\n"
        + "        backend = str(self.config.get(\"standard_execution_backend\", \"\")).strip()\n"
        + "        if backend not in {\"moveit\", \"moveit_sim\"}:\n"
        + "            return\n"
        + f"        from {domain_pkg}.devices.{device_id}.moveit_commissioning import (\n"
        + "            build_robot_moveit_binding,\n"
        + "        )\n\n"
        + "        model = type(self)._device_registry_meta[\"model\"]\n"
        + "        binding, client = build_robot_moveit_binding(\n"
        + "            device_id=self.device_id,\n"
        + "            ros_node=ros_node,\n"
        + "            model_config=model,\n"
        + "            node_config=self.config,\n"
        + "        )\n"
        + "        self._moveit_split_binding = binding\n"
        + "        self._moveit_client = client\n"
    )
    updated = updated.replace(init_match.group(1), insert, 1)
    return updated, updated != source


def _default_point_set_v3_py() -> str:
    return '''"""{domain_pkg} 最小 PointSet v3 解析。"""

from __future__ import annotations

import hashlib
from collections.abc import Mapping
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml
from unilab_arm_cr5 import MODEL_DESCRIPTOR as ARM_MODEL
from unilab_rail_linear import MODEL_DESCRIPTOR as RAIL_MODEL
from unilab_robot_contracts import (
    InstallationCalibration,
    RigidTransform,
    RobotPointSetResolver,
    ToolContext,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ASSET_ROOT = PACKAGE_ROOT.parent / "deployment" / "robot_cell" / "assets"
POINT_SET_PATH = ASSET_ROOT / "point_sets" / "{point_set_basename}.v3.yaml"
TOOL_CONTEXT_ID = "{tool_context_id}"


def placeholder_tool() -> ToolContext:
    digest = hashlib.sha256(TOOL_CONTEXT_ID.encode("utf-8")).hexdigest()
    return ToolContext(TOOL_CONTEXT_ID, digest, RigidTransform.identity(), 1)


@lru_cache(maxsize=1)
def load_point_set_document() -> dict[str, Any]:
    document = yaml.safe_load(POINT_SET_PATH.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"PointSet 不是对象: {{POINT_SET_PATH}}")
    return document


def make_resolver(point_set: Mapping[str, Any]) -> RobotPointSetResolver:
    calibration_ref = point_set["installation_calibration"]
    calibration = InstallationCalibration(
        str(calibration_ref["revision"]),
        str(calibration_ref["digest"]),
        {{}},
    )
    return RobotPointSetResolver(
        point_set,
        arm_model=ARM_MODEL,
        tool_context=placeholder_tool(),
        calibration=calibration,
        rail_model=RAIL_MODEL,
    )


@lru_cache(maxsize=1)
def robot_point_set_resolver() -> RobotPointSetResolver:
    return make_resolver(load_point_set_document())
'''


def _default_moveit_commissioning_py() -> str:
    return '''"""{domain_pkg} 在已有 move_group 上绑定 CR5 调试客户端。"""

from __future__ import annotations

import hashlib
from typing import Any

COMMISSIONING_MOTION_PROFILE_REF = "{boot_id_prefix}-manual-step@1.0.0"
HARDWARE_PROFILE_DIGEST = hashlib.sha256(
    b"{boot_id_prefix}-moveit-sim-profile"
).hexdigest()
TOOL_CONTEXT_DIGEST = "{tool_context_digest}"


def robot_point_set_resolver() -> Any:
    from {domain_pkg}.robot_cell.point_set_v3 import robot_point_set_resolver as _resolver

    return _resolver()


def build_robot_moveit_binding(
    *,
    device_id: str,
    ros_node: Any,
    model_config: Any,
    node_config: Any = None,
) -> tuple[Any, Any]:
    from unilab_arm_cr5 import create_moveit_commissioning_adapter
    from unilab_robot_contracts import DeploymentMode
    from unilab_robot_runtime import bind_commissioning_runtime

    normalized = str(device_id).strip()
    client, qualified = create_robot_moveit_client(
        device_id=normalized,
        ros_node=ros_node,
        model_config=model_config,
        node_config=node_config,
    )
    resolver = robot_point_set_resolver()
    adapter = create_moveit_commissioning_adapter(
        moveit_client=client,
        targets=resolver.arm_targets,
        qualified_joint_names=qualified,
        point_set_revision=resolver.revision,
        hardware_profile_digest=HARDWARE_PROFILE_DIGEST,
        tool_context_digest=TOOL_CONTEXT_DIGEST,
        commissioning_velocity_limit=0.20,
        commissioning_acceleration_limit=0.20,
    )
    binding = bind_commissioning_runtime(
        adapter,
        frozenset({{f"moveit:{{normalized}}:arm"}}),
        owner_id=f"{boot_id_prefix}:{{normalized}}:commissioning",
        deployment_mode=DeploymentMode.SIMULATION,
    )
    return binding, client


def create_robot_moveit_client(
    *,
    device_id: str,
    ros_node: Any,
    model_config: Any,
    node_config: Any = None,
) -> tuple[Any, tuple[str, ...]]:
    from unilab_arm_cr5 import build_joint_state_name_map
    from unilabos.device_mesh.package_moveit_model import (
        create_package_moveit_client,
        load_package_moveit_model,
    )

    normalized = str(device_id).strip()
    if not normalized:
        raise ValueError("MoveIt 客户端缺少 device_id")
    bundle = load_package_moveit_model(
        model_config,
        {{
            "id": normalized,
            "position": {{}},
            "config": dict(node_config or {{}}),
        }},
    )
    client = create_package_moveit_client(ros_node, bundle)
    qualified = build_joint_state_name_map(device_id=normalized).qualified_joint_names
    return client, qualified
'''


def _default_rail_simulation_py() -> str:
    return '''"""{domain_pkg} 导轨仿真驱动：发布完全限定关节名。"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from typing import Any

_PUBLISH_PERIOD_S = 0.02
_DEFAULT_VELOCITY_M_S = 0.2
_SIMULATION_BACKENDS = frozenset({{"simulation", "mock"}})
_AXIS_LOCK = threading.RLock()
_AXES: dict[str, "RailSimulationDriver"] = {{}}


def simulation_backends() -> frozenset[str]:
    return _SIMULATION_BACKENDS


def register_rail_simulation_axis(device_id: str, driver: "RailSimulationDriver") -> None:
    key = str(device_id).strip()
    if not key:
        raise ValueError("导轨仿真轴必须包含 Graph device_id")
    with _AXIS_LOCK:
        _AXES[key] = driver


def require_rail_simulation_axis(device_id: str) -> "RailSimulationDriver":
    key = str(device_id).strip()
    if not key:
        raise RuntimeError("导轨仿真轴查找缺少 Graph device_id")
    with _AXIS_LOCK:
        driver = _AXES.get(key)
    if driver is None:
        raise RuntimeError(f"物理图父设备未登记导轨仿真轴: {{key}}")
    return driver


class RailSimulationDriver:
    def __init__(
        self,
        *,
        device_id: str,
        sleep: Callable[[float], None] = time.sleep,
        velocity_m_s: float = _DEFAULT_VELOCITY_M_S,
    ) -> None:
        from unilab_rail_linear import build_joint_state_name_map

        self._device_id = str(device_id).strip()
        self._sleep = sleep
        self._lock = threading.RLock()
        self._velocity_m_s = float(velocity_m_s)
        self._name_map = build_joint_state_name_map(device_id=self._device_id)
        self._position_si = 0.0
        self._publisher: Any = None
        self._joint_state_type: Any = None
        self._ros_node: Any = None

    @property
    def qualified_joint_names(self) -> tuple[str, ...]:
        return self._name_map.qualified_joint_names

    def joint_state_frame(self) -> tuple[tuple[str, ...], tuple[float, ...]]:
        with self._lock:
            return self.qualified_joint_names, (self._position_si,)

    def attach_ros(self, ros_node: object) -> None:
        from sensor_msgs.msg import JointState

        self._ros_node = ros_node
        self._joint_state_type = JointState
        self._publisher = ros_node.create_publisher(JointState, "/joint_states", 10)
        callback_group = getattr(ros_node, "callback_group", None)
        create_timer = ros_node.create_timer
        if callback_group is None:
            create_timer(_PUBLISH_PERIOD_S, self.publish_joint_state)
        else:
            create_timer(
                _PUBLISH_PERIOD_S,
                self.publish_joint_state,
                callback_group=callback_group,
            )
        self.publish_joint_state()

    def publish_joint_state(self) -> None:
        if self._publisher is None or self._joint_state_type is None:
            return
        names, positions = self.joint_state_frame()
        message = self._joint_state_type()
        clock = getattr(self._ros_node, "get_clock", None)
        if clock is not None:
            message.header.stamp = clock().now().to_msg()
        message.name = list(names)
        message.position = list(positions)
        message.velocity = [0.0] * len(names)
        self._publisher.publish(message)

    def move_to_si(self, position_si: float) -> None:
        from unilab_rail_linear import MODEL_DESCRIPTOR

        target = float(position_si)
        if not math.isfinite(target):
            raise ValueError("导轨 SI 目标必须是有限数值")
        lower, upper = MODEL_DESCRIPTOR.travel_m
        if not lower <= target <= upper:
            raise ValueError(
                f"导轨目标超出型号行程 {{MODEL_DESCRIPTOR.travel_m}}: {{target}}"
            )
        with self._lock:
            start = self._position_si
        self._interpolate(start, target)

    def _interpolate(self, start: float, end: float) -> None:
        distance = abs(end - start)
        duration = distance / self._velocity_m_s if self._velocity_m_s > 0 else 0.0
        if duration <= 0:
            with self._lock:
                self._position_si = end
            self.publish_joint_state()
            return
        steps = max(1, int(duration / _PUBLISH_PERIOD_S))
        for index in range(1, steps + 1):
            self._sleep(duration / steps)
            ratio = index / steps
            with self._lock:
                self._position_si = start + (end - start) * ratio
            self.publish_joint_state()
'''
