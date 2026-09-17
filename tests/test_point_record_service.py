"""PointSet 示教落盘单测（无 MoveIt）。"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from unilab_robot_runtime.device_card.observation_types import ExecutorObservation
from unilab_robot_runtime.device_card.point_record_service import teach_joint_target_from_observation


def test_teach_joint_target_from_observation_bumps_revision(tmp_path: Path) -> None:
    point_set = {
        "schema": "unilab.robot-point-set/v3",
        "revision": "demo@1.0.0",
        "global": {
            "arm": {
                "home": {
                    "type": "joint_positions",
                    "value": [0.0, 0.0],
                    "source_point": "P1",
                }
            }
        },
    }
    path = tmp_path / "demo.v3.yaml"
    path.write_text(yaml.safe_dump(point_set, sort_keys=False), encoding="utf-8")
    obs: ExecutorObservation = {
        "online": True,
        "idle": True,
        "joint_positions_si": [0.5, 0.6],
    }
    result = teach_joint_target_from_observation(
        obs,
        point_set_path=path,
        target_ref="global.arm.home",
        joint_count=2,
        confirm=True,
    )
    assert result["target_ref"] == "global.arm.home"
    assert result["target_revision"] != "demo@1.0.0"
    reloaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    values = reloaded["global"]["arm"]["home"]["value"]
    assert len(values) == 2
