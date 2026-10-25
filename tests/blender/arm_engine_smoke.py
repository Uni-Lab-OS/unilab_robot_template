"""Blender 本地冒烟：OS ``Scene`` + ``BlenderArmEngine`` 的关节写入/读回。

运行（Blender 5.x，Windows）::

    D:\\blender\\blender.exe --background --factory-startup \
        --python tests/blender/arm_engine_smoke.py

脚本自行加入本仓 packages 与同级 Uni-Lab-OS 的导入路径；不修改 OS 仓。
Blender 的 Python 通常不带 PyYAML，而 ``unilab_robot_runtime.__init__`` 的
聚合导入需要它，因此仅在缺 yaml 时只加载 ``engines`` 子包并打印说明。
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
OS_ROOT = Path(
    os.environ.get("UNILAB_OS_ROOT", str(REPO_ROOT.parent / "Uni-Lab-OS"))
)

JOINT_NAMES = (
    "cr7_joint_1",
    "cr7_joint_2",
    "cr7_joint_3",
    "cr7_joint_4",
    "cr7_joint_5",
    "cr7_joint_6",
)
JOINT_VALUES = (0.10, -0.20, 0.30, -0.40, 0.50, -0.60)
IDENTITY = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)


def _install_paths() -> None:
    """把本仓 packages 与 OS 仓加入 sys.path（Blender 忽略 PYTHONPATH）。"""

    candidates = [OS_ROOT, *sorted((REPO_ROOT / "packages").glob("*/src"))]
    for path in reversed(candidates):
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)


def _load_engine_class():
    """优先正常导入；只在 Blender 缺 PyYAML 时只加载 engines 子包。"""

    try:
        from unilab_robot_runtime.engines import BlenderArmEngine

        return BlenderArmEngine
    except ModuleNotFoundError as error:
        if error.name != "yaml":
            raise
        print(
            "ARM_ENGINE_SMOKE_NOTE: runtime __init__ 聚合导入需要 PyYAML；"
            "Blender 内只加载 engines 子包"
        )

    import types

    runtime_src = (
        REPO_ROOT / "packages" / "unilab-robot-runtime" / "src" / "unilab_robot_runtime"
    )
    package = types.ModuleType("unilab_robot_runtime")
    package.__path__ = [str(runtime_src)]
    package.__package__ = "unilab_robot_runtime"
    sys.modules["unilab_robot_runtime"] = package
    from unilab_robot_runtime.engines import BlenderArmEngine

    return BlenderArmEngine


def _assert_close(readback: dict[str, float], expected: tuple[float, ...]) -> None:
    """逐轴核对读回值，任何偏差都作为冒烟失败。"""

    for name, value in zip(JOINT_NAMES, expected, strict=True):
        actual = readback[name]
        if not math.isclose(actual, value, rel_tol=1e-9, abs_tol=1e-6):
            raise AssertionError(f"{name} 读回 {actual!r} != {value!r}")


def main() -> int:
    """创建真实 OS Scene，写入一组关节姿态、读回并验证逐点轨迹。"""

    _install_paths()
    try:
        from unilabos.simulation.blender_kinematic import Scene
    except Exception as error:  # noqa: BLE001 - 记录原始错误后退出
        print(f"ARM_ENGINE_SMOKE_FAIL: 无法导入 OS Scene: {error!r}")
        return 1

    try:
        BlenderArmEngine = _load_engine_class()
        scene = Scene()
        for name in JOINT_NAMES:
            scene.create(f"joint/{name}", IDENTITY)
        engine = BlenderArmEngine(
            scene,
            {name: f"joint/{name}" for name in JOINT_NAMES},
        )

        engine.apply_joint_positions(JOINT_NAMES, JOINT_VALUES)
        _assert_close(engine.read_joint_positions(), JOINT_VALUES)

        engine.apply_trajectory(
            JOINT_NAMES,
            (JOINT_VALUES, (0.0,) * len(JOINT_NAMES)),
            (0.0, 0.05),
        )
        _assert_close(engine.read_joint_positions(), (0.0,) * len(JOINT_NAMES))

        try:
            engine.attach("cup", "tcp")
        except NotImplementedError:
            pass
        else:
            raise AssertionError("BlenderArmEngine.attach 默认应明确不支持")
    except Exception as error:  # noqa: BLE001 - 冒烟必须打印原始错误
        import traceback

        traceback.print_exc()
        print(f"ARM_ENGINE_SMOKE_FAIL: {error!r}")
        return 1

    print(
        "ARM_ENGINE_SMOKE_OK "
        f"joints={len(JOINT_NAMES)} scene={type(scene).__module__}."
        f"{type(scene).__name__}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
