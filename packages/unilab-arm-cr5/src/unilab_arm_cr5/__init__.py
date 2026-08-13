"""CR5 模块的稳定工厂与可选 standalone 包装。"""

from .arm_module import ArmModule
from .factory import (
    MODEL_DESCRIPTOR,
    MODULE_API_VERSION,
    MODULE_KIND,
    MODULE_VERSION,
    create_arm_module,
    create_moveit_backend,
    create_moveit_commissioning_adapter,
    create_plc_backend,
    create_tcp_sdk_backend,
)
from .moveit_model import MoveItModelBundle, build_moveit_model
from .standalone_device import StandaloneArmDevice

__version__ = MODULE_VERSION

__all__ = [
    "MODEL_DESCRIPTOR",
    "MODULE_API_VERSION",
    "MODULE_KIND",
    "MODULE_VERSION",
    "ArmModule",
    "MoveItModelBundle",
    "StandaloneArmDevice",
    "build_moveit_model",
    "create_arm_module",
    "create_moveit_backend",
    "create_moveit_commissioning_adapter",
    "create_plc_backend",
    "create_tcp_sdk_backend",
]
