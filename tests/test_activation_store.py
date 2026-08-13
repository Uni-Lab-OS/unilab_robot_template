"""D14 候选准备、原子激活和任务冻结的持久化测试。"""

from __future__ import annotations

from pathlib import Path

import pytest
from unilab_robot_contracts import (
    SiteAccessActivation,
    SiteTopologySnapshot,
    TopologySite,
)
from unilab_robot_runtime import RobotActivationStore


def _activation(site_uuid: str, epoch: str) -> SiteAccessActivation:
    """构造一个已完成全部引用校验的候选激活。"""

    declarations = {
        "schema": "unilab.site-access-declarations/v1",
        "revision": f"access@{epoch}",
        "declarations": [
            {
                "declaration_ref": "rack-r1c1",
                "owner_resource_ref": "rack",
                "slot_key": "R1C1",
                "allowed_payload_profiles": ["beaker@v1"],
                "occupancy_observation_ref": "occupancy:rack:R1C1",
                "operations": {
                    "pick": {
                        "kind": "point_target",
                        "ref": "rack.r1c1",
                        "motion_policy_ref": "standard",
                        "qualification_ref": "site:rack:pick",
                    }
                },
            }
        ],
    }
    return SiteAccessActivation.prepare(
        declarations,
        declarations_digest=("a" if epoch == "one" else "c") * 64,
        topology=SiteTopologySnapshot(
            epoch,
            ("b" if epoch == "one" else "d") * 64,
            (TopologySite(site_uuid, "rack", "R1C1"),),
        ),
        point_target_refs={"rack.r1c1"},
    )


def test_activation_switch_requires_idle_and_retains_old_until_settlement(
    tmp_path: Path,
) -> None:
    """旧激活必须持续保留到使用它的任务完成物理结算。"""

    store = RobotActivationStore(tmp_path / "activation.sqlite3")
    first = _activation("site-first", "one")
    second = _activation("site-second", "two")
    store.prepare(first)
    store.prepare(second)
    store.activate(
        first.activation_id,
        has_inflight_claims=lambda: False,
        has_unsettled_fence=lambda: False,
    )
    lease = store.freeze_for_task("task-1")
    assert lease.activation_id == first.activation_id

    store.activate(
        second.activation_id,
        has_inflight_claims=lambda: False,
        has_unsettled_fence=lambda: False,
    )
    assert store.retained_activation_ids() == {
        first.activation_id,
        second.activation_id,
    }
    assert store.purge_unreferenced() == 0

    with pytest.raises(RuntimeError, match="物理结算"):
        store.settle_task("task-1", physical_settlement=False)
    store.settle_task("task-1", physical_settlement=True)
    assert store.purge_unreferenced() == 1
    assert store.retained_activation_ids() == {second.activation_id}


def test_activation_rejects_claim_or_fence(tmp_path: Path) -> None:
    """执行 Claim 或 UNKNOWN Fence 存在时不得切换候选。"""

    store = RobotActivationStore(tmp_path / "activation.sqlite3")
    activation = _activation("site-first", "one")
    store.prepare(activation)
    with pytest.raises(RuntimeError, match="Claim"):
        store.activate(
            activation.activation_id,
            has_inflight_claims=lambda: True,
            has_unsettled_fence=lambda: False,
        )
    with pytest.raises(RuntimeError, match="Fence"):
        store.activate(
            activation.activation_id,
            has_inflight_claims=lambda: False,
            has_unsettled_fence=lambda: True,
        )
