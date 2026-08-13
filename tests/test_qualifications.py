"""D11 三类资格不可替代及生产模式关闭失败测试。"""

from __future__ import annotations

import pytest
from unilab_robot_contracts import ActionKind, DeploymentMode, QualificationCatalog


def _catalog() -> QualificationCatalog:
    """返回一个仿真点位和 Site 操作资格集合。"""

    digest = "a" * 64
    return QualificationCatalog.from_mapping(
        {
            "schema": "unilab.robot-qualifications/v1",
            "points": [
                {
                    "qualification_ref": "point:s04",
                    "point_set_digest": digest,
                    "target_ref": "s04-process.main",
                    "resolved_target_digest": digest,
                    "arm_model_digest": digest,
                    "tool_context_digest": digest,
                    "installation_calibration_digest": digest,
                    "collision_environment_digest": digest,
                    "evidence_ref": "evidence://sim/s04",
                    "approved_by": "simulation-suite",
                    "approved_for": ["simulation"],
                }
            ],
            "programs": [],
            "site_operations": [
                {
                    "qualification_ref": "site:s04:pick",
                    "declarations_digest": digest,
                    "declaration_ref": "s04-process",
                    "action": "pick",
                    "payload_profiles": ["beaker@v1"],
                    "execution_qualification_ref": "point:s04",
                    "motion_profiles_digest": digest,
                    "occupancy_observation_ref": "occupancy:s04:S041",
                    "evidence_ref": "evidence://sim/s04-pick",
                    "approved_by": "simulation-suite",
                    "approved_for": ["simulation"],
                }
            ],
        }
    )


def test_site_operation_requires_its_own_and_base_qualification() -> None:
    """Site 端到端资格必须引用另一个 Point/Program 资格。"""

    result = _catalog().require_site_operation(
        "site:s04:pick",
        mode=DeploymentMode.SIMULATION,
        declarations_digest="a" * 64,
        declaration_ref="s04-process",
        action=ActionKind.PICK,
        payload_profile="beaker@v1",
        occupancy_observation_ref="occupancy:s04:S041",
        motion_profiles_digest="a" * 64,
    )
    assert result.execution_qualification_ref == "point:s04"


def test_simulation_evidence_cannot_enable_production() -> None:
    """仿真证据不得被同一个 YAML 字段冒充为生产验收。"""

    with pytest.raises(ValueError, match="production"):
        _catalog().require_site_operation(
            "site:s04:pick",
            mode=DeploymentMode.PRODUCTION,
            declarations_digest="a" * 64,
            declaration_ref="s04-process",
            action=ActionKind.PICK,
            payload_profile="beaker@v1",
            occupancy_observation_ref="occupancy:s04:S041",
            motion_profiles_digest="a" * 64,
        )


def test_site_qualification_rejects_payload_or_declaration_digest_drift() -> None:
    """负载或声明资产变化后必须重新做 Site 操作资格确认。"""

    with pytest.raises(ValueError, match="不匹配"):
        _catalog().require_site_operation(
            "site:s04:pick",
            mode=DeploymentMode.SIMULATION,
            declarations_digest="b" * 64,
            declaration_ref="s04-process",
            action=ActionKind.PICK,
            payload_profile="vial@v1",
            occupancy_observation_ref="occupancy:s04:S041",
            motion_profiles_digest="a" * 64,
        )


def test_site_qualification_rejects_motion_profile_drift() -> None:
    """D10 策略摘要变化后不得复用旧 SiteOperationQualification。"""

    with pytest.raises(ValueError, match="MotionProfile"):
        _catalog().require_site_operation(
            "site:s04:pick",
            mode=DeploymentMode.SIMULATION,
            declarations_digest="a" * 64,
            declaration_ref="s04-process",
            action=ActionKind.PICK,
            payload_profile="beaker@v1",
            occupancy_observation_ref="occupancy:s04:S041",
            motion_profiles_digest="b" * 64,
        )


def test_point_qualification_rejects_any_planning_asset_drift() -> None:
    """模型、工具、标定、碰撞环境或解析目标变化均使点位资格失效。"""

    with pytest.raises(ValueError, match="规划环境"):
        _catalog().require_point(
            "point:s04",
            mode=DeploymentMode.SIMULATION,
            point_set_digest="a" * 64,
            target_ref="s04-process.main",
            resolved_target_digest="a" * 64,
            arm_model_digest="a" * 64,
            tool_context_digest="a" * 64,
            installation_calibration_digest="a" * 64,
            collision_environment_digest="b" * 64,
        )
