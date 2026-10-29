"""候选机器人资产的准备、空闲切换和任务冻结持久化。"""

from __future__ import annotations

import json
import sqlite3
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from unilab_robot_contracts import SiteAccessActivation


@dataclass(frozen=True, slots=True)
class FrozenRobotActivation:
    """一个 WorkflowTask 在物理结算前必须持续持有的激活身份。"""

    task_ref: str
    activation_id: str


class RobotActivationStore:
    """以 SQLite 原子管理 prepare→activate→freeze→settle 生命周期。"""

    def __init__(self, path: str | Path) -> None:
        """创建持久化表；初始化本身不激活候选资产。"""

        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._initialize()

    def prepare(self, activation: SiteAccessActivation) -> str:
        """持久化已完全校验的候选快照，重复相同身份保持幂等。"""

        payload = json.dumps(
            {
                "activation_id": activation.activation_id,
                "declarations_revision": activation.declarations_revision,
                "declarations_digest": activation.declarations_digest,
                "topology_epoch": activation.topology_epoch,
                "topology_digest": activation.topology_digest,
                "bindings": {
                    key: {
                        "site_uuid": value.site_ref.site_uuid,
                        "owner_resource_ref": value.locator.owner_resource_ref,
                        "slot_key": value.locator.slot_key,
                    }
                    for key, value in sorted(activation.bindings.items())
                },
            },
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        with self._connect() as connection:
            existing = connection.execute(
                "SELECT payload FROM activations WHERE activation_id = ?",
                (activation.activation_id,),
            ).fetchone()
            if existing is not None and str(existing[0]) != payload:
                raise RuntimeError("相同 activation_id 已存在不同载荷")
            connection.execute(
                "INSERT OR IGNORE INTO activations "
                "(activation_id, payload, created_at) VALUES (?, ?, ?)",
                (activation.activation_id, payload, time.time()),
            )
        return activation.activation_id

    def activate(
        self,
        activation_id: str,
        *,
        has_inflight_claims: Callable[[], bool],
        has_unsettled_fence: Callable[[], bool],
    ) -> None:
        """仅在无执行 Claim/Fence 时原子切换当前激活。

        参数：已准备身份及两个权威安全查询。返回：无。异常：候选缺失、
        正在执行或存在未知物理状态时拒绝；不会触发机械臂或导轨运动。
        """

        if has_inflight_claims():
            raise RuntimeError("存在执行中 WorkflowTask Claim，禁止切换机器人资产")
        if has_unsettled_fence():
            raise RuntimeError("存在未物理结算 Fence，禁止切换机器人资产")
        with self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            candidate = connection.execute(
                "SELECT 1 FROM activations WHERE activation_id = ?",
                (activation_id,),
            ).fetchone()
            if candidate is None:
                raise ValueError(f"机器人候选激活尚未 prepare: {activation_id}")
            connection.execute(
                "INSERT INTO active_activation (singleton, activation_id) VALUES (1, ?) "
                "ON CONFLICT(singleton) DO UPDATE SET activation_id = excluded.activation_id",
                (activation_id,),
            )
            connection.commit()

    def freeze_for_task(self, task_ref: str) -> FrozenRobotActivation:
        """在任务开始时冻结当前激活；同一任务重试返回原身份。"""

        if not task_ref.strip():
            raise ValueError("task_ref 不能为空")
        with self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            existing = connection.execute(
                "SELECT activation_id FROM task_activation_leases WHERE task_ref = ?",
                (task_ref,),
            ).fetchone()
            if existing is not None:
                connection.commit()
                return FrozenRobotActivation(task_ref, str(existing[0]))
            active = connection.execute(
                "SELECT activation_id FROM active_activation WHERE singleton = 1"
            ).fetchone()
            if active is None:
                raise RuntimeError("尚无活动机器人资产，禁止创建任务")
            activation_id = str(active[0])
            connection.execute(
                "INSERT INTO task_activation_leases "
                "(task_ref, activation_id, created_at) VALUES (?, ?, ?)",
                (task_ref, activation_id, time.time()),
            )
            connection.commit()
        return FrozenRobotActivation(task_ref, activation_id)

    def settle_task(self, task_ref: str, *, physical_settlement: bool) -> None:
        """只在收到物理结算证据后释放任务对旧激活的保留。"""

        if not physical_settlement:
            raise RuntimeError("未物理结算的任务不得释放机器人激活")
        with self._connect() as connection:
            deleted = connection.execute(
                "DELETE FROM task_activation_leases WHERE task_ref = ?",
                (task_ref,),
            ).rowcount
            if deleted != 1:
                raise ValueError(f"任务未持有机器人激活: {task_ref}")

    def current_activation_id(self) -> str | None:
        """返回当前公共激活身份；未激活时返回 None。"""

        with self._connect() as connection:
            row = connection.execute(
                "SELECT activation_id FROM active_activation WHERE singleton = 1"
            ).fetchone()
        return None if row is None else str(row[0])

    def retained_activation_ids(self) -> frozenset[str]:
        """返回当前激活及仍被未结算任务冻结的旧激活。"""

        with self._connect() as connection:
            rows = connection.execute(
                "SELECT activation_id FROM active_activation "
                "UNION SELECT activation_id FROM task_activation_leases"
            ).fetchall()
        return frozenset(str(row[0]) for row in rows)

    def purge_unreferenced(self) -> int:
        """删除非当前且无任务冻结的候选或旧激活，返回删除数量。"""

        retained = self.retained_activation_ids()
        with self._connect() as connection:
            if not retained:
                return connection.execute("DELETE FROM activations").rowcount
            placeholders = ",".join("?" for _ in retained)
            return connection.execute(
                f"DELETE FROM activations WHERE activation_id NOT IN ({placeholders})",
                tuple(sorted(retained)),
            ).rowcount

    def _initialize(self) -> None:
        """建立三个最小表并启用 WAL 与外键。"""

        with self._connect() as connection:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS activations (
                    activation_id TEXT PRIMARY KEY,
                    payload TEXT NOT NULL,
                    created_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS active_activation (
                    singleton INTEGER PRIMARY KEY CHECK (singleton = 1),
                    activation_id TEXT NOT NULL REFERENCES activations(activation_id)
                );
                CREATE TABLE IF NOT EXISTS task_activation_leases (
                    task_ref TEXT PRIMARY KEY,
                    activation_id TEXT NOT NULL REFERENCES activations(activation_id),
                    created_at REAL NOT NULL
                );
                """
            )

    def _connect(self) -> sqlite3.Connection:
        """创建启用外键、WAL 和 busy timeout 的短事务连接。"""

        connection = sqlite3.connect(self.path, timeout=5.0, isolation_level=None)
        connection.execute("PRAGMA foreign_keys = ON")
        connection.execute("PRAGMA journal_mode = WAL")
        connection.execute("PRAGMA busy_timeout = 5000")
        return connection


__all__ = ["FrozenRobotActivation", "RobotActivationStore"]
