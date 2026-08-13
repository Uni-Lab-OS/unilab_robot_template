"""D8 作者键到当前 OS Site UUID 的一次性激活测试。"""

from __future__ import annotations

import pytest
from unilab_robot_contracts import (
    SiteAccessActivation,
    SiteTopologySnapshot,
    TopologySite,
    parse_site_access_declarations,
)


def _declarations() -> dict[str, object]:
    """返回不含环境实例 UUID 的两种后端作者声明。"""

    return {
        "schema": "unilab.site-access-declarations/v1",
        "revision": "szlab-access@2.0.0",
        "declarations": [
            {
                "declaration_ref": "s04-process",
                "owner_resource_ref": "s04_process_warehouse",
                "slot_key": "S041",
                "allowed_payload_profiles": ["beaker_500ml@v1"],
                "occupancy_observation_ref": "occupancy:s04:S041",
                "operations": {
                    "pick": {
                        "kind": "point_target",
                        "ref": "s04-process.main",
                        "motion_policy_ref": "standard-access",
                        "qualification_ref": "site:s04:pick",
                    },
                    "place": {
                        "kind": "plc_program",
                        "ref": "S04.place",
                        "parameters": {"position": 1},
                        "qualification_ref": "site:s04:place",
                    },
                },
            }
        ],
    }


def _topology(site_uuid: str, epoch: str) -> SiteTopologySnapshot:
    """生成 OS 当前启动周期的 Site 拓扑。"""

    return SiteTopologySnapshot(
        epoch,
        "b" * 64,
        (TopologySite(site_uuid, "s04_process_warehouse", "S041"),),
    )


def test_activation_resolves_current_uuid_but_author_source_stays_stable() -> None:
    """相同作者键在 OS 重建后可解析为新 UUID，不需要改领域包。"""

    first = SiteAccessActivation.prepare(
        _declarations(),
        declarations_digest="a" * 64,
        topology=_topology("site-uuid-first", "boot-1"),
        point_target_refs={"s04-process.main"},
        plc_program_refs={"S04.place"},
    )
    second = SiteAccessActivation.prepare(
        _declarations(),
        declarations_digest="a" * 64,
        topology=_topology("site-uuid-second", "boot-2"),
        point_target_refs={"s04-process.main"},
        plc_program_refs={"S04.place"},
    )

    assert first.bindings["s04-process"].site_ref.site_uuid == "site-uuid-first"
    assert second.bindings["s04-process"].site_ref.site_uuid == "site-uuid-second"
    assert first.activation_id != second.activation_id
    assert first.binding_for_site("site-uuid-first").locator.slot_key == "S041"


def test_activation_fails_for_zero_or_multiple_topology_matches() -> None:
    """作者键零匹配或多匹配时不得猜测 Site。"""

    with pytest.raises(ValueError, match="实际 0 次"):
        SiteAccessActivation.prepare(
            _declarations(),
            declarations_digest="a" * 64,
            topology=SiteTopologySnapshot("boot", "b" * 64, ()),
            point_target_refs={"s04-process.main"},
            plc_program_refs={"S04.place"},
        )
    duplicate = SiteTopologySnapshot(
        "boot",
        "b" * 64,
        (
            TopologySite("site-1", "s04_process_warehouse", "S041"),
            TopologySite("site-2", "s04_process_warehouse", "S041"),
        ),
    )
    with pytest.raises(ValueError, match="实际 2 次"):
        SiteAccessActivation.prepare(
            _declarations(),
            declarations_digest="a" * 64,
            topology=duplicate,
            point_target_refs={"s04-process.main"},
            plc_program_refs={"S04.place"},
        )


def test_author_declaration_rejects_instance_uuid_and_old_binding_schema() -> None:
    """领域设备包不得继续保存 UUID 或旧 Warehouse/Site 绑定。"""

    value = _declarations()
    declaration = value["declarations"][0]  # type: ignore[index]
    declaration["owner_resource_ref"] = "123e4567-e89b-12d3-a456-426614174000"  # type: ignore[index]
    with pytest.raises(ValueError, match="UUID"):
        parse_site_access_declarations(value)

    old = {
        "schema": "unilab.site-bindings/v0",
        "revision": "old",
        "bindings": [],
    }
    with pytest.raises(ValueError, match="declarations/v1"):
        parse_site_access_declarations(old)
