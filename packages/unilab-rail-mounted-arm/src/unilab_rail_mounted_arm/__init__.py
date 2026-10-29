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
from .planning_coordinator import (
    PlannedPhase,
    PlanningCoordinator,
    WaypointValidator,
    WorkCellPlanResult,
)
from .simulation import SimulationInterlockProvider, SimulationRailMountedArmRuntime
from .workcell_device import RailMountedArmWorkCellDevice

__all__ = [
    "CompositeModelInput",
    "ConfiguredInterlockProvider",
    "EndpointLease",
    "EndpointLeaseRegistry",
    "InterlockBinding",
    "InterlockVariablePort",
    "PlannedPhase",
    "PlanningCoordinator",
    "SpatialInterlockBinding",
    "SpatiallyGuardedInterlockProvider",
    "RailMountedArmCoordinator",
    "RailMountedArmWorkCellDevice",
    "SimulationInterlockProvider",
    "SimulationRailMountedArmRuntime",
    "WaypointValidator",
    "WorkCellPlanResult",
    "compile_xacro_snapshot",
]
