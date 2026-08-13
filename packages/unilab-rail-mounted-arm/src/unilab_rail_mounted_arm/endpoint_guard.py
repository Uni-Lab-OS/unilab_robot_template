"""公共 Device 激活阶段的物理端点唯一所有权门禁。"""

from __future__ import annotations

from dataclasses import dataclass
from threading import RLock

from unilab_robot_contracts import EndpointConflictError


@dataclass
class EndpointLease:
    """一组 endpoint 的可释放激活租约。"""

    _registry: EndpointLeaseRegistry
    owner_id: str
    endpoint_ids: frozenset[str]
    _released: bool = False

    def close(self) -> None:
        """幂等释放本 Device 的所有 endpoint。"""

        if not self._released:
            self._registry.release(self.owner_id, self.endpoint_ids)
            self._released = True


class EndpointLeaseRegistry:
    """拒绝 standalone、rail 与 WorkCell 重复激活同一物理端点。"""

    def __init__(self) -> None:
        """创建空的进程内激活注册表。"""

        self._owners: dict[str, str] = {}
        self._lock = RLock()

    def acquire(self, owner_id: str, endpoint_ids: frozenset[str]) -> EndpointLease:
        """原子取得全部端点；任何冲突都不产生部分占用。"""

        if not owner_id.strip() or not endpoint_ids:
            raise ValueError("endpoint lease 必须包含 owner_id 与 endpoint_ids")
        with self._lock:
            conflicts = {
                endpoint: self._owners[endpoint]
                for endpoint in endpoint_ids
                if endpoint in self._owners
            }
            if conflicts:
                raise EndpointConflictError(
                    f"物理端点已被公共 Device 激活: {conflicts}"
                )
            for endpoint in endpoint_ids:
                self._owners[endpoint] = owner_id
        return EndpointLease(self, owner_id, endpoint_ids)

    def release(self, owner_id: str, endpoint_ids: frozenset[str]) -> None:
        """只释放仍属于同一 owner 的端点，避免误删其他租约。"""

        with self._lock:
            for endpoint in endpoint_ids:
                if self._owners.get(endpoint) == owner_id:
                    del self._owners[endpoint]
