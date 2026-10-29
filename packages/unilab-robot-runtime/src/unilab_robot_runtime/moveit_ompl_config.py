"""MoveIt OMPL 管线配置守卫：单数 ``planning_plugin`` 必须显式且为 OMPL。

MoveIt ≤2.5.x 的 ``planning_pipeline`` 先读单数 ``planning_plugin``；键缺失时
pluginlib 按声明顺序取第一个插件，本机声明顺序恰好是 CHOMP。MoveIt 2.10+ 的
生成参数监听器只读复数 ``planning_plugins``。只写一边都会让另一个版本静默
换掉插件，而 CHOMP 不支持位姿目标，失败又容易被误读成"不可达"。

因此两个后端构造参数时都调用本守卫：单数必须是
``ompl_interface/OMPLPlanner``，复数（若给出）必须与单数一致；缺省、只写复数、
显式 CHOMP 或两边不一致都在构造期抛 ``OMPLConfigurationError``，绝不把可能
回落 CHOMP 的配置交给 MoveIt。
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

#: 唯一允许的 OMPL 规划插件名（pluginlib 全名）。
OMPL_PLANNING_PLUGIN = "ompl_interface/OMPLPlanner"
#: 缺省时按 pluginlib 顺序可能被选中的插件；出现在错误信息里便于排查。
CHOMP_PLANNING_PLUGIN = "chomp_interface/CHOMPPlanner"


class OMPLConfigurationError(ValueError):
    """规划管线配置会静默回落到 CHOMP 或未知插件。"""


def _plugin_names(value: object) -> tuple[str, ...]:
    """把复数键规范化成插件名元组；不可迭代时返回空元组。"""

    if isinstance(value, str):
        return (value,)
    if isinstance(value, (bytes, bytearray)):
        return (bytes(value).decode("utf-8", "replace"),)
    try:
        return tuple(str(item) for item in value)  # type: ignore[arg-type]
    except TypeError:
        return ()


def require_ompl_planning_plugin(
    pipeline: Mapping[str, Any], *, context: str = "ompl"
) -> str:
    """校验一个 MoveIt 管线参数块确实选择 OMPL，返回单数插件名。

    ``pipeline`` 是 ``move_group`` 参数里的 ``ompl:`` 子树，或 MoveItPy
    ``config_dict`` 里的 ``ompl`` 块。只接受单数
    ``planning_plugin=ompl_interface/OMPLPlanner``；若同时给出复数
    ``planning_plugins``，它必须恰好是同一插件。任何会导致 CHOMP 被静默选中的
    形态都抛 ``OMPLConfigurationError``。
    """

    if not isinstance(pipeline, Mapping):
        raise TypeError(f"{context} 配置必须是映射")
    singular = pipeline.get("planning_plugin")
    plural = pipeline.get("planning_plugins")
    if singular is None:
        if plural is None:
            raise OMPLConfigurationError(
                f"{context}: 缺少单数 planning_plugin；MoveIt ≤2.5.x 会按 "
                f"pluginlib 声明顺序静默选择插件（本机顺序是 CHOMP）"
            )
        raise OMPLConfigurationError(
            f"{context}: 只提供了复数 planning_plugins；MoveIt ≤2.5.x 只读单数 "
            f"planning_plugin，会静默回落到 CHOMP，必须显式写 "
            f"planning_plugin={OMPL_PLANNING_PLUGIN}"
        )
    if not isinstance(singular, str) or singular.strip() != OMPL_PLANNING_PLUGIN:
        if str(singular).strip() == CHOMP_PLANNING_PLUGIN:
            raise OMPLConfigurationError(
                f"{context}: planning_plugin 是 CHOMP；本层只接受 "
                f"{OMPL_PLANNING_PLUGIN}，CHOMP 不支持位姿目标"
            )
        raise OMPLConfigurationError(
            f"{context}: planning_plugin 必须是 {OMPL_PLANNING_PLUGIN}，"
            f"收到 {singular!r}"
        )
    if plural is not None:
        names = _plugin_names(plural)
        if names != (OMPL_PLANNING_PLUGIN,):
            raise OMPLConfigurationError(
                f"{context}: planning_plugins 必须恰好是 "
                f"[{OMPL_PLANNING_PLUGIN}]（MoveIt 2.10+ 只读复数键），"
                f"收到 {names!r}；不能与单数 planning_plugin 不一致"
            )
    return singular.strip()


__all__ = [
    "CHOMP_PLANNING_PLUGIN",
    "OMPLConfigurationError",
    "OMPL_PLANNING_PLUGIN",
    "require_ompl_planning_plugin",
]
