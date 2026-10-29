"""规则阵列点位和直接导轨位置的确定性解析合同。"""

from __future__ import annotations

import math
import re
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Protocol

from .geometry import Quaternion, Vector3
from .targets import numeric_sequence

_CELL_REF = re.compile(r"^r(?P<row>[1-9][0-9]*)c(?P<col>[1-9][0-9]*)$")


class RailTargetModel(Protocol):
    """PointSet v3 校验直接导轨位置所需的最小型号接口。"""

    model_ref: str
    travel_m: tuple[float, float]


@dataclass(frozen=True, slots=True)
class ResolvedGridCell:
    """从三个锚点确定性生成的一个设备局部 TCP 交互点。"""

    cell_key: str
    xyz_m: Vector3
    orientation_xyzw: Quaternion
    rail_position_si: float | None
    source_chain: tuple[str, ...]


def resolve_affine_grid(
    group_ref: str,
    raw: Mapping[str, Any],
    *,
    rail_model: RailTargetModel | None,
) -> Mapping[str, ResolvedGridCell]:
    """解析 `affine_grid/v1` 三锚点阵列和稀疏修正。

    参数：设备分组引用、阵列配置和可选精确导轨型号。返回：以 1-based
    `rNcM` 为键的全部解析单元。异常：缺锚、修正锚点、越界 cell、重叠
    rail group 或型号不兼容时关闭失败。解析只返回内存结果，不创建第二份
    可编辑逐格点位资产。
    """

    if str(raw.get("type", "")) != "affine_grid/v1":
        raise ValueError(f"{group_ref}.grid.type 必须为 affine_grid/v1")
    rows = _positive_int(raw.get("rows"), f"{group_ref}.grid.rows")
    cols = _positive_int(raw.get("cols"), f"{group_ref}.grid.cols")
    if rows < 2 or cols < 2:
        raise ValueError("affine_grid/v1 的 rows/cols 必须都不小于 2")
    frame_ref = str(raw.get("frame_ref", "")).strip()
    if not frame_ref.startswith("device:"):
        raise ValueError(f"{group_ref}.grid.frame_ref 必须是 device: 局部坐标系")

    first = "r1c1"
    last_row = f"r{rows}c1"
    last_cell = f"r{rows}c{cols}"
    anchor_keys = (first, last_row, last_cell)
    anchors_raw = _mapping(raw.get("anchors"), f"{group_ref}.grid.anchors")
    if set(anchors_raw) != set(anchor_keys):
        raise ValueError(
            f"{group_ref}.grid.anchors 必须精确包含 {sorted(anchor_keys)}"
        )
    anchor_items = {
        key: _mapping(anchors_raw[key], f"{group_ref}.grid.anchors.{key}")
        for key in anchor_keys
    }
    anchors = {
        key: _vector3(
            item.get("xyz_m"),
            f"{group_ref}.grid.anchors.{key}.xyz_m",
        )
        for key, item in anchor_items.items()
    }
    anchor_orientations = {
        key: _quaternion(
            item["orientation_override_xyzw"],
            f"{group_ref}.grid.anchors.{key}.orientation_override_xyzw",
        )
        for key, item in anchor_items.items()
        if "orientation_override_xyzw" in item
    }
    orientation = _quaternion(
        raw.get("orientation_xyzw"),
        f"{group_ref}.grid.orientation_xyzw",
    )
    corrections_raw = _mapping(
        raw.get("corrections", {}),
        f"{group_ref}.grid.corrections",
    )
    corrections: dict[str, tuple[Vector3, Quaternion | None]] = {}
    for cell_key, value in corrections_raw.items():
        normalized_key = str(cell_key)
        _cell_coordinates(normalized_key, rows=rows, cols=cols)
        if normalized_key in anchor_keys:
            raise ValueError(f"权威锚点 {normalized_key} 禁止重复声明 correction")
        item = _mapping(value, f"{group_ref}.grid.corrections.{normalized_key}")
        residual = _vector3(
            item.get("xyz_m", (0.0, 0.0, 0.0)),
            f"{group_ref}.grid.corrections.{normalized_key}.xyz_m",
        )
        orientation_override = (
            None
            if "orientation_override_xyzw" not in item
            else _quaternion(
                item["orientation_override_xyzw"],
                f"{group_ref}.grid.corrections."
                f"{normalized_key}.orientation_override_xyzw",
            )
        )
        corrections[normalized_key] = (residual, orientation_override)

    rail_positions = _resolve_rail_assignments(
        group_ref,
        raw.get("rail"),
        rows=rows,
        cols=cols,
        rail_model=rail_model,
    )
    anchor_a = anchors[first]
    anchor_b = anchors[last_row]
    anchor_c = anchors[last_cell]
    resolved: dict[str, ResolvedGridCell] = {}
    for row in range(1, rows + 1):
        for col in range(1, cols + 1):
            cell_key = f"r{row}c{col}"
            u = (row - 1) / (rows - 1)
            v = (col - 1) / (cols - 1)
            base = tuple(
                anchor_a[index]
                + u * (anchor_b[index] - anchor_a[index])
                + v * (anchor_c[index] - anchor_b[index])
                for index in range(3)
            )
            residual, correction_orientation = corrections.get(
                cell_key,
                ((0.0, 0.0, 0.0), None),
            )
            final_xyz = tuple(
                base[index] + residual[index] for index in range(3)
            )
            sources = [
                f"{group_ref}.grid:affine_grid/v1",
                *(f"{group_ref}.grid.anchors.{key}" for key in anchor_keys),
            ]
            if cell_key in corrections:
                sources.append(f"{group_ref}.grid.corrections.{cell_key}")
            if cell_key in anchor_orientations:
                sources.append(
                    f"{group_ref}.grid.anchors.{cell_key}.orientation_override_xyzw"
                )
            resolved[cell_key] = ResolvedGridCell(
                cell_key,
                final_xyz,
                anchor_orientations.get(cell_key)
                or correction_orientation
                or orientation,
                rail_positions.get(cell_key),
                tuple(sources),
            )
    return resolved


def _resolve_rail_assignments(
    group_ref: str,
    value: Any,
    *,
    rows: int,
    cols: int,
    rail_model: RailTargetModel | None,
) -> Mapping[str, float]:
    """按 override > group > default 解析全部阵列导轨直接位置。"""

    if value is None:
        return {}
    if rail_model is None:
        raise ValueError(f"{group_ref}.grid 声明 rail，但 PointSet 未装配导轨")
    raw = _mapping(value, f"{group_ref}.grid.rail")
    if "default" not in raw:
        raise ValueError(f"{group_ref}.grid.rail.default 不能为空")
    default = _rail_position(raw["default"], rail_model, "rail.default")
    assignments = {
        f"r{row}c{col}": default
        for row in range(1, rows + 1)
        for col in range(1, cols + 1)
    }
    grouped_cells: set[str] = set()
    groups = raw.get("groups", ())
    if isinstance(groups, (str, bytes)) or not isinstance(groups, list):
        raise TypeError(f"{group_ref}.grid.rail.groups 必须是列表")
    for index, group_value in enumerate(groups):
        group = _mapping(group_value, f"{group_ref}.grid.rail.groups[{index}]")
        if set(group) != {"position", "cells"}:
            raise ValueError("rail group 只能包含 position 与 cells，且不得命名")
        position = _rail_position(
            group["position"],
            rail_model,
            f"rail.groups[{index}].position",
        )
        cells = group.get("cells")
        if isinstance(cells, (str, bytes)) or not isinstance(cells, list):
            raise TypeError(f"rail.groups[{index}].cells 必须是列表")
        for cell_value in cells:
            cell_key = str(cell_value)
            _cell_coordinates(cell_key, rows=rows, cols=cols)
            if cell_key in grouped_cells:
                raise ValueError(f"rail groups 重复覆盖 cell: {cell_key}")
            grouped_cells.add(cell_key)
            assignments[cell_key] = position
    overrides = _mapping(raw.get("overrides", {}), "rail.overrides")
    for cell_value, position_value in overrides.items():
        cell_key = str(cell_value)
        _cell_coordinates(cell_key, rows=rows, cols=cols)
        assignments[cell_key] = _rail_position(
            position_value,
            rail_model,
            f"rail.overrides.{cell_key}",
        )
    return assignments


def _rail_position(value: Any, model: RailTargetModel, field: str) -> float:
    """验证一个直接 SI 导轨位置为有限值且位于型号行程内。"""

    try:
        position = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} 必须是直接 SI 位置") from exc
    if not math.isfinite(position):
        raise ValueError(f"{field} 必须是有限数")
    lower, upper = model.travel_m
    if not lower <= position <= upper:
        raise ValueError(f"{field}={position} 超出 RailModel 行程 {model.travel_m}")
    return position


def _cell_coordinates(cell_key: str, *, rows: int, cols: int) -> tuple[int, int]:
    """解析并验证一个稳定 1-based 阵列 cell key。"""

    matched = _CELL_REF.fullmatch(cell_key)
    if matched is None:
        raise ValueError(f"非法 grid cell key: {cell_key}")
    row, col = int(matched.group("row")), int(matched.group("col"))
    if row > rows or col > cols:
        raise ValueError(f"grid cell 越界: {cell_key}")
    return row, col


def _positive_int(value: Any, field: str) -> int:
    """要求字段是非布尔正整数。"""

    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError(f"{field} 必须是正整数")
    return value


def _vector3(value: Any, field: str) -> Vector3:
    """解析三维 SI 向量。"""

    values = numeric_sequence(value, field)
    if len(values) != 3:
        raise ValueError(f"{field} 必须包含三个有限数")
    return values  # type: ignore[return-value]


def _quaternion(value: Any, field: str) -> Quaternion:
    """解析完整绝对 XYZW 单位四元数。"""

    values = numeric_sequence(value, field)
    if len(values) != 4:
        raise ValueError(f"{field} 必须包含四个有限数")
    norm = math.sqrt(sum(item * item for item in values))
    if not math.isclose(norm, 1.0, rel_tol=0.0, abs_tol=1e-6):
        raise ValueError(f"{field} 必须是单位四元数")
    return values  # type: ignore[return-value]


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    """要求一个阵列字段是对象。"""

    if not isinstance(value, Mapping):
        raise TypeError(f"{field} 必须是对象")
    return value


__all__ = ["RailTargetModel", "ResolvedGridCell", "resolve_affine_grid"]
