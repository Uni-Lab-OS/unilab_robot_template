"""厂商和领域无关的 Robotics 运行时组合入口。"""

from .access_motion_backend import AccessMotionBackend
from .activation_store import FrozenRobotActivation, RobotActivationStore
from .binding import MaintenanceSession, RuntimeBinding, build_test_runtime
from .factory import (
    RuntimeDependencies,
    RuntimeRequirements,
    arm_model_descriptor,
    create_runtime,
    load_point_set,
    runtime_requirements,
)
from .manipulation import ManipulationSequence, ManipulationSequenceRunner
from .point_maintenance import (
    PointMaintenanceService,
    PointSetQualification,
    PointTestEvidence,
    PublishedPointSet,
    ValidatedPointSet,
)

__version__ = "0.1.0"

__all__ = [
    "AccessMotionBackend",
    "FrozenRobotActivation",
    "MaintenanceSession",
    "ManipulationSequence",
    "ManipulationSequenceRunner",
    "PointMaintenanceService",
    "PointSetQualification",
    "PointTestEvidence",
    "PublishedPointSet",
    "RobotActivationStore",
    "RuntimeBinding",
    "RuntimeDependencies",
    "RuntimeRequirements",
    "ValidatedPointSet",
    "arm_model_descriptor",
    "build_test_runtime",
    "create_runtime",
    "load_point_set",
    "runtime_requirements",
]
