"""通用单轴导轨模块及可选 standalone 包装。"""

from .factory import (
    MODEL_DESCRIPTOR,
    MODULE_API_VERSION,
    MODULE_KIND,
    MODULE_VERSION,
    create_plc_module,
    create_simulation_module,
)
from .rail_module import RailAxisPort, RailModule
from .standalone_device import StandaloneRailDevice

__version__ = MODULE_VERSION

__all__ = [
    "MODEL_DESCRIPTOR",
    "MODULE_API_VERSION",
    "MODULE_KIND",
    "MODULE_VERSION",
    "RailAxisPort",
    "RailModule",
    "StandaloneRailDevice",
    "create_plc_module",
    "create_simulation_module",
]
