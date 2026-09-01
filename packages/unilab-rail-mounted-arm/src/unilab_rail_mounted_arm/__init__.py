"""导轨机械臂通用协调包；不包含任何厂商模型。"""

from .coordinator import RailMountedArmCoordinator
from .endpoint_guard import EndpointLease, EndpointLeaseRegistry
from .interlock_adapter import (
    ConfiguredInterlockProvider,
    InterlockBinding,
    InterlockVariablePort,
    SpatialInterlockBinding,
    SpatiallyGuardedInterlockProvider,
)
from .model_compiler import CompositeModelInput, compile_xacro_snapshot
from .simulation import SimulationInterlockProvider, SimulationRailMountedArmRuntime
from .workcell_device import RailMountedArmWorkCellDevice

__all__ = [
    "CompositeModelInput",
    "ConfiguredInterlockProvider",
    "EndpointLease",
    "EndpointLeaseRegistry",
    "InterlockBinding",
    "InterlockVariablePort",
    "SpatialInterlockBinding",
    "SpatiallyGuardedInterlockProvider",
    "RailMountedArmCoordinator",
    "RailMountedArmWorkCellDevice",
    "SimulationInterlockProvider",
    "SimulationRailMountedArmRuntime",
    "compile_xacro_snapshot",
]
