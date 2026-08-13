"""Arm/Rail 型号资产与部署安装变换的离线静态模型编译器。"""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class CompositeModelInput:
    """原子引用两个型号模型和安装变换；更换型号只替换对应 ref。"""

    arm_model_ref: str
    arm_model_digest: str
    rail_model_ref: str
    rail_model_digest: str
    mount_xyz_rpy: tuple[float, float, float, float, float, float]


def compile_xacro_snapshot(spec: CompositeModelInput, output: str | Path) -> str:
    """生成普通静态 xacro snapshot 并返回内容摘要。

    参数：exact 模型引用、digest、安装变换与输出路径。返回：SHA-256。异常：字段缺失时拒绝生成。
    """

    required = (
        spec.arm_model_ref,
        spec.arm_model_digest,
        spec.rail_model_ref,
        spec.rail_model_digest,
    )
    if any(not value.strip() for value in required):
        raise ValueError("组合模型必须使用 exact refs 与 digests")
    xyz = " ".join(str(value) for value in spec.mount_xyz_rpy[:3])
    rpy = " ".join(str(value) for value in spec.mount_xyz_rpy[3:])
    content = (
        '<?xml version="1.0"?>\n'
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="rail_mounted_arm">\n'
        f"  <!-- rail={spec.rail_model_ref}@sha256:{spec.rail_model_digest} -->\n"
        f"  <!-- arm={spec.arm_model_ref}@sha256:{spec.arm_model_digest} -->\n"
        f'  <xacro:include filename="{spec.rail_model_ref}"/>\n'
        f'  <xacro:include filename="{spec.arm_model_ref}"/>\n'
        f'  <joint name="arm_mount" type="fixed"><origin xyz="{xyz}" rpy="{rpy}"/>'
        '<parent link="rail_carriage"/><child link="arm_base"/></joint>\n'
        "</robot>\n"
    )
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")
    return hashlib.sha256(content.encode("utf-8")).hexdigest()
