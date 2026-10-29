"""三轴/导轨有序轴链的顺序、限位、单位、frame 与稳定摘要合同。"""

from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest
from unilab_robot_contracts import (
    LinearAxis,
    LinearAxisChain,
    linear_axis_chain_digest,
)


def _axes() -> tuple[LinearAxis, ...]:
    """返回一条合法的三轴链，轴序本身就是合同的一部分。"""

    return (
        LinearAxis("rail_x", "m", (-0.1, 1.2), "rail_base"),
        LinearAxis("rail_y", "m", (0.0, 0.5), "rail_base"),
        LinearAxis("rail_z", "m", (0.0, 0.3), "gantry"),
    )


def _chain() -> LinearAxisChain:
    """返回合法三轴链，避免每个用例重复构造。"""

    return LinearAxisChain(_axes())


def test_chain_keeps_declared_axis_order() -> None:
    """链必须按声明顺序保留轴，且暴露同名有序视图。"""

    chain = _chain()

    assert chain.axes == _axes()
    assert chain.names == ("rail_x", "rail_y", "rail_z")
    assert [axis.lower for axis in chain.axes] == [-0.1, 0.0, 0.0]
    assert [axis.upper for axis in chain.axes] == [1.2, 0.5, 0.3]


def test_chain_digest_is_stable_and_order_sensitive() -> None:
    """同一显式输入必须得到同一摘要；交换轴序必须改变摘要。"""

    first = _chain()
    second = LinearAxisChain(tuple(_axes()))
    reordered = LinearAxisChain(tuple(reversed(_axes())))

    assert first.digest == second.digest
    assert first.digest == linear_axis_chain_digest(first)
    assert len(first.digest) == 64
    assert set(first.digest) <= set("0123456789abcdef")
    assert reordered.digest != first.digest


def test_chain_digest_covers_limits_unit_and_frame() -> None:
    """限位、单位或 frame 任一显式输入变化都必须改变摘要。"""

    base = _chain()
    original = _axes()
    variants = (
        LinearAxisChain(
            (LinearAxis("rail_x", "m", (-0.1, 1.3), "rail_base"), *original[1:])
        ),
        LinearAxisChain(
            (LinearAxis("rail_x", "rad", (-0.1, 1.2), "rail_base"), *original[1:])
        ),
        LinearAxisChain(
            (LinearAxis("rail_x", "m", (-0.1, 1.2), "other_base"), *original[1:])
        ),
    )

    assert len({base.digest, *(variant.digest for variant in variants)}) == 4


@pytest.mark.parametrize(
    ("limits", "match"),
    [
        ((0.0, float("nan")), "有限数"),
        ((float("-inf"), 0.0), "有限数"),
        ((True, 1.0), "有限数"),
        ((1.0, 0.0), "下限"),
        ((0.0, 1.0, 2.0), "两个"),
    ],
)
def test_axis_rejects_invalid_limits(limits: tuple[float, ...], match: str) -> None:
    """限位必须成对、有限且 lower <= upper。"""

    with pytest.raises(ValueError, match=match):
        LinearAxis("rail_x", "m", limits, "rail_base")


@pytest.mark.parametrize("unit", ["mm", "", "degree"])
def test_axis_rejects_unsupported_unit(unit: str) -> None:
    """单位只接受 SI 的 m/rad，不做隐式换算。"""

    with pytest.raises(ValueError, match="m/rad"):
        LinearAxis("rail_x", unit, (0.0, 1.0), "rail_base")


def test_axis_rejects_empty_name_or_frame() -> None:
    """轴名与 frame 都是运动学身份，不允许空白。"""

    with pytest.raises(ValueError, match="name"):
        LinearAxis("", "m", (0.0, 1.0), "rail_base")
    with pytest.raises(ValueError, match="frame"):
        LinearAxis("rail_x", "m", (0.0, 1.0), " ")


def test_chain_rejects_duplicate_or_empty_axis_sequence() -> None:
    """有序链不得有重复轴名，也不允许空链。"""

    with pytest.raises(ValueError, match="重复"):
        LinearAxisChain((LinearAxis("rail_x", "m", (0.0, 1.0), "rail_base"),) * 2)
    with pytest.raises(ValueError, match="至少"):
        LinearAxisChain(())


def test_chain_rejects_non_axis_and_non_sequence_input() -> None:
    """链只接受有序 LinearAxis 序列，不做隐式转换。"""

    with pytest.raises(TypeError, match="LinearAxis"):
        LinearAxisChain((object(),))
    with pytest.raises(TypeError, match="序列"):
        LinearAxisChain({_axes()[0]})


def test_chain_and_axis_are_frozen() -> None:
    """轴与链都是冻结值对象，摘要不能被事后改写。"""

    chain = _chain()

    with pytest.raises(FrozenInstanceError):
        chain.axes = ()  # type: ignore[misc]
    with pytest.raises(FrozenInstanceError):
        chain.axes[0].unit = "rad"  # type: ignore[misc]
