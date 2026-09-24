"""OS ``create_package_moveit_client`` 与现有 MoveIt2 客户端之间的兼容层。"""

from __future__ import annotations

from typing import Any


def package_moveit_client_factory(*args: Any, **kwargs: Any) -> Any:
    """构造 MoveIt2 客户端并补上 ``configure_model_link_references``。"""

    from unilabos.devices.ros_dev.moveit2 import MoveIt2

    client = MoveIt2(*args, **kwargs)
    link_references: dict[str, str] = {}

    def configure_model_link_references(references: dict[str, str]) -> None:
        link_references.clear()
        link_references.update(dict(references))

    client.configure_model_link_references = configure_model_link_references  # type: ignore[attr-defined]
    client.model_link_references = link_references  # type: ignore[attr-defined]
    return client


__all__ = ["package_moveit_client_factory"]
