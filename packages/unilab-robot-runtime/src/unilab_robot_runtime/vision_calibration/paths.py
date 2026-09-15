"""视觉标定资产路径（领域注入）。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class VisionCalibrationPaths:
    calibration_path: Path
    registry_path: Path
    registry_revision: str
    calibration_revision: str
    tool_context_ref: str
