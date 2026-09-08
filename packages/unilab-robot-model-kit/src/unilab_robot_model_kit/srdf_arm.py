"""SRDF helpers for six-axis MoveIt arm models."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from collections.abc import Sequence


def build_srdf_arm_chain(
    *,
    robot_name: str,
    prefix: str,
    planning_group: str,
    base_link: str,
    tip_link: str,
    disabled_collisions: Sequence[tuple[str, str, str]],
) -> str:
    """Build SRDF with one arm chain and disable-collision pairs."""

    root = ET.Element("robot", {"name": robot_name})
    group = ET.SubElement(root, "group", {"name": planning_group})
    ET.SubElement(
        group,
        "chain",
        {
            "base_link": f"{prefix}{base_link}",
            "tip_link": f"{prefix}{tip_link}",
        },
    )
    for left, right, reason in disabled_collisions:
        ET.SubElement(
            root,
            "disable_collisions",
            {
                "link1": f"{prefix}{left}",
                "link2": f"{prefix}{right}",
                "reason": reason,
            },
        )
    return ET.tostring(root, encoding="unicode")
