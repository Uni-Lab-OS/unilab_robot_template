"""型号 provider 的版本、摘要、关节顺序、限位与 TCP 合同。"""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from .geometry import RigidTransform

_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_JOINT_UNITS = frozenset({"rad", "m"})
_SCHEMA = "unilab.model-provider/v1"


@dataclass(frozen=True, slots=True)
class ModelProviderDescriptor:
    """一个型号 provider 的稳定身份和运动学事实。

    ``digest`` 只由 ``model_provider_digest`` 的显式输入（型号名、版本、
    关节顺序、限位、TCP 与来源内容摘要）决定，不包含时间、随机数或本机
    安装路径；同一输入在任何机器上复算得到同一摘要。
    """

    name: str
    version: str
    digest: str
    joint_names: tuple[str, ...]
    joint_limits: tuple[tuple[float, float], ...]
    tcp: RigidTransform
    model_path: str | None = None
    joint_units: tuple[str, ...] = ()
    base_frame: str = ""
    tip_frame: str = ""

    def __post_init__(self) -> None:
        """校验稳定身份、有序关节和逐轴限位。"""

        name = str(self.name).strip()
        version = str(self.version).strip()
        if not name:
            raise ValueError("ModelProviderDescriptor.name 不能为空")
        if not version:
            raise ValueError("ModelProviderDescriptor.version 不能为空")
        if not _DIGEST.fullmatch(str(self.digest)):
            raise ValueError("ModelProviderDescriptor.digest 必须是 SHA-256")
        if not isinstance(self.tcp, RigidTransform):
            raise ValueError("ModelProviderDescriptor.tcp 必须是 RigidTransform")
        joint_names = _normalize_joint_names(self.joint_names)
        joint_limits = _normalize_joint_limits(self.joint_limits, joint_names)
        joint_units = tuple(str(unit).strip() for unit in self.joint_units)
        if joint_units:
            if len(joint_units) != len(joint_names):
                raise ValueError("ModelProviderDescriptor.joint_units 必须逐轴声明")
            unsupported = set(joint_units) - _JOINT_UNITS
            if unsupported:
                raise ValueError(
                    "ModelProviderDescriptor.joint_units 只允许 rad/m: "
                    + ", ".join(sorted(unsupported))
                )
        model_path = (
            None if self.model_path is None else str(self.model_path).strip()
        )
        if model_path is not None and not model_path:
            raise ValueError("ModelProviderDescriptor.model_path 不能为空字符串")
        object.__setattr__(self, "name", name)
        object.__setattr__(self, "version", version)
        object.__setattr__(self, "digest", str(self.digest))
        object.__setattr__(self, "joint_names", joint_names)
        object.__setattr__(self, "joint_limits", joint_limits)
        object.__setattr__(self, "joint_units", joint_units)
        object.__setattr__(self, "model_path", model_path)
        object.__setattr__(self, "base_frame", str(self.base_frame).strip())
        object.__setattr__(self, "tip_frame", str(self.tip_frame).strip())


def model_provider_digest(
    *,
    name: str,
    version: str,
    joint_names: Sequence[str],
    joint_limits: Sequence[tuple[float, float]],
    tcp: RigidTransform,
    sources: Mapping[str, str],
) -> str:
    """由显式输入稳定计算型号 provider 摘要。

    参数：``sources`` 是来源标识到内容 SHA-256 的映射（例如 ``urdf``、
    ``model``）；不接收路径、时间或随机数。返回：64 位小写十六进制摘要。
    异常：名称/版本为空、关节或限位非法、TCP 类型错误或来源摘要非法时拒绝。
    安全：摘要只证明"这组型号事实"，不替代调用方对锁定来源摘要的校验。
    """

    normalized_name = str(name).strip()
    normalized_version = str(version).strip()
    if not normalized_name:
        raise ValueError("model_provider_digest.name 不能为空")
    if not normalized_version:
        raise ValueError("model_provider_digest.version 不能为空")
    if not isinstance(tcp, RigidTransform):
        raise ValueError("model_provider_digest.tcp 必须是 RigidTransform")
    names = _normalize_joint_names(joint_names)
    limits = _normalize_joint_limits(joint_limits, names)
    if not sources:
        raise ValueError("model_provider_digest 必须声明来源内容摘要")
    normalized_sources: dict[str, str] = {}
    for source, value in sources.items():
        key = str(source).strip()
        digest = str(value).strip()
        if not key:
            raise ValueError("model_provider_digest 来源标识不能为空")
        if not _DIGEST.fullmatch(digest):
            raise ValueError(f"model_provider_digest 来源 {key} 必须是 SHA-256")
        normalized_sources[key] = digest
    payload = {
        "schema": _SCHEMA,
        "name": normalized_name,
        "version": normalized_version,
        "joint_names": list(names),
        "joint_limits": [[lower, upper] for lower, upper in limits],
        "tcp": {
            "xyz_m": list(tcp.translation_m),
            "orientation_xyzw": list(tcp.orientation_xyzw),
        },
        "sources": normalized_sources,
    }
    encoded = json.dumps(
        payload,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    )
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _normalize_joint_names(names: Sequence[str]) -> tuple[str, ...]:
    """把有序关节名规范为去空白的非重复 tuple。"""

    normalized = tuple(str(name).strip() for name in names)
    if not normalized:
        raise ValueError("型号 provider 必须声明至少一个关节")
    if any(not name for name in normalized):
        raise ValueError("型号 provider 关节名不能为空")
    if len(set(normalized)) != len(normalized):
        raise ValueError("型号 provider 关节名不得重复")
    return normalized


def _normalize_joint_limits(
    limits: Sequence[tuple[float, float]],
    names: tuple[str, ...],
) -> tuple[tuple[float, float], ...]:
    """把限位规范为与关节顺序一一对应的有限上下限。"""

    normalized: list[tuple[float, float]] = []
    if len(limits) != len(names):
        raise ValueError("型号 provider 限位必须逐轴声明")
    for name, limit in zip(names, limits, strict=True):
        lower, upper = (float(value) for value in limit)
        if not math.isfinite(lower) or not math.isfinite(upper):
            raise ValueError(f"型号 provider 关节 {name} 限位必须是有限数")
        if lower > upper:
            raise ValueError(f"型号 provider 关节 {name} 下限不得大于上限")
        normalized.append((lower, upper))
    return tuple(normalized)


__all__ = ["ModelProviderDescriptor", "model_provider_digest"]
