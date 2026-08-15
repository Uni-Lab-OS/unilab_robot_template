"""通用单轴导轨模块及可选 standalone 包装。"""

from .factory import (
    MODEL_DESCRIPTOR,
    MODULE_API_VERSION,
    MODULE_KIND,
    MODULE_VERSION,
    build_joint_state_name_map,
    create_plc_module,
    create_simulation_module,
)
from .kinematic_model import RailKinematicModelBundle, build_kinematic_model
from .rail_module import RailAxisPort, RailModule
from .standalone_device import StandaloneRailDevice

__version__ = MODULE_VERSION

__all__ = [
    "MODEL_DESCRIPTOR",
    "MODULE_API_VERSION",
    "MODULE_KIND",
    "MODULE_VERSION",
    "RailAxisPort",
    "RailKinematicModelBundle",
    "RailModule",
    "StandaloneRailDevice",
    "build_joint_state_name_map",
    "build_kinematic_model",
    "create_plc_module",
    "create_simulation_module",
]
