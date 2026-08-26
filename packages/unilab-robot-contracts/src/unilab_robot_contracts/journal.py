"""命令幂等、UNKNOWN 栅栏与物理结算的持久边界合同。"""

from __future__ import annotations

import json
import sqlite3
from dataclasses import dataclass
from pathlib import Path
from threading import RLock
from time import time
from collections.abc import Mapping
from typing import Any, Protocol

from .commands import (
    CommandResult,
    CommandState,
    PhysicalSettlementEvidence,
    RailMoveCommand,
    RobotCommand,
)
from .errors import CommandRejectedError


@dataclass(frozen=True)
class JournalRecord:
    """不可变命令摘要与最近权威结果。"""

    fingerprint: str
    result: CommandResult
    fenced: bool


class CommandJournal(Protocol):
    """协调器与 standalone 模块共用的命令存储端口。"""

    def accept(
        self, command: RobotCommand | RailMoveCommand
    ) -> tuple[bool, CommandResult]:
        """首次接受命令或返回精确重放；同 id 不同内容必须拒绝。"""

    def update(
        self, result: CommandResult, *, fenced: bool | None = None
    ) -> CommandResult:
        """更新命令投影；UNKNOWN 默认保留栅栏。"""

    def get(self, command_id: str) -> CommandResult | None:
        """读取命令投影，不触发物理动作。"""

    def is_fenced(self, command_id: str) -> bool:
        """返回命令是否仍持有禁止盲目重放的栅栏（Fence）。"""

    def fenced_command_ids(self) -> tuple[str, ...]:
        """返回全部未完成物理结算的命令身份。"""

    def settle(
        self, result: CommandResult, evidence: PhysicalSettlementEvidence
    ) -> CommandResult:
        """仅用精确物理见证解除 execution_unknown 阻断。"""


class InMemoryCommandJournal:
    """用于测试和仿真的确定性命令账本；生产可替换为 SQLite adapter。"""

    def __init__(self) -> None:
        """创建空账本；所有写入通过同一进程锁串行化。"""

        self._records: dict[str, JournalRecord] = {}
        self._lock = RLock()

    def accept(
        self, command: RobotCommand | RailMoveCommand
    ) -> tuple[bool, CommandResult]:
        """持久接受命令，保证 command_id 的精确幂等。"""

        fingerprint = command.fingerprint()
        with self._lock:
            existing = self._records.get(command.command_id)
            if existing is not None:
                if existing.fingerprint != fingerprint:
                    raise CommandRejectedError("command_id 已绑定不同命令内容")
                return False, existing.result
            result = CommandResult(
                command.command_id, CommandState.ACCEPTED, "命令已持久接受"
            )
            self._records[command.command_id] = JournalRecord(
                fingerprint, result, False
            )
            return True, result

    def update(
        self, result: CommandResult, *, fenced: bool | None = None
    ) -> CommandResult:
        """更新已接受命令；execution_unknown 自动建立 Fence。"""

        with self._lock:
            existing = self._records.get(result.command_id)
            if existing is None:
                raise KeyError(f"命令尚未接受: {result.command_id}")
            _validate_transition(existing.result.state, result.state, fenced)
            next_fence = existing.fenced if fenced is None else fenced
            if result.state is CommandState.EXECUTION_UNKNOWN:
                next_fence = True
            if result.state.terminal and fenced is None and not existing.fenced:
                next_fence = False
            self._records[result.command_id] = JournalRecord(
                existing.fingerprint, result, next_fence
            )
            return result

    def get(self, command_id: str) -> CommandResult | None:
        """返回最近结果；找不到时返回 ``None``。"""

        with self._lock:
            record = self._records.get(command_id)
            return None if record is None else record.result

    def is_fenced(self, command_id: str) -> bool:
        """返回命令的 Fence 状态。"""

        with self._lock:
            record = self._records.get(command_id)
            return bool(record and record.fenced)

    def fenced_command_ids(self) -> tuple[str, ...]:
        """按命令身份排序返回全部 Fence，供新派发关闭失败。"""

        with self._lock:
            return tuple(sorted(key for key, value in self._records.items() if value.fenced))

    def settle(
        self, result: CommandResult, evidence: PhysicalSettlementEvidence
    ) -> CommandResult:
        """验证命令、终态和见证后显式解除阻断。"""

        _validate_settlement(result, evidence)
        with self._lock:
            existing = self._records.get(result.command_id)
            if existing is None:
                raise KeyError(f"命令尚未接受: {result.command_id}")
            if existing.result.state is not CommandState.EXECUTION_UNKNOWN:
                raise CommandRejectedError("只允许显式结算 execution_unknown")
            self._records[result.command_id] = JournalRecord(
                existing.fingerprint, result, False
            )
        return result


class SQLiteCommandJournal:
    """生产可用的 SQLite 命令账本；重启把未结算命令固定为 execution_unknown。"""

    def __init__(self, path: str | Path) -> None:
        """创建数据库并对重启前未结算命令建立 Fence。"""

        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = RLock()
        with self._connect() as connection:
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS robot_commands_v1 (
                    command_id TEXT PRIMARY KEY,
                    fingerprint TEXT NOT NULL,
                    state TEXT NOT NULL,
                    message TEXT NOT NULL,
                    output_json TEXT NOT NULL,
                    fenced INTEGER NOT NULL
                )
                """
            )
            connection.execute(
                """
                UPDATE robot_commands_v1
                SET state = ?, message = ?, fenced = 1
                WHERE state IN (?, ?)
                """,
                (
                    CommandState.EXECUTION_UNKNOWN.value,
                    "进程重启后无法证明先前物理执行结果",
                    CommandState.ACCEPTED.value,
                    CommandState.RUNNING.value,
                ),
            )

    def _connect(self) -> sqlite3.Connection:
        """返回启用 Row 投影的短连接。"""

        connection = sqlite3.connect(self.path, timeout=10.0)
        connection.row_factory = sqlite3.Row
        return connection

    def accept(
        self, command: RobotCommand | RailMoveCommand
    ) -> tuple[bool, CommandResult]:
        """原子接受新命令或返回精确重放；同 id 不同摘要拒绝。"""

        fingerprint = command.fingerprint()
        with self._lock, self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            row = connection.execute(
                "SELECT * FROM robot_commands_v1 WHERE command_id = ?",
                (command.command_id,),
            ).fetchone()
            if row is not None:
                if str(row["fingerprint"]) != fingerprint:
                    raise CommandRejectedError("command_id 已绑定不同命令内容")
                return False, _result_from_row(row)
            result = CommandResult(
                command.command_id, CommandState.ACCEPTED, "命令已持久接受"
            )
            connection.execute(
                "INSERT INTO robot_commands_v1 VALUES (?, ?, ?, ?, ?, 0)",
                (
                    command.command_id,
                    fingerprint,
                    result.state.value,
                    result.message,
                    "{}",
                ),
            )
            return True, result

    def update(
        self, result: CommandResult, *, fenced: bool | None = None
    ) -> CommandResult:
        """更新命令状态；UNKNOWN 自动保留 Fence，物理终态默认释放。"""

        with self._lock, self._connect() as connection:
            row = connection.execute(
                "SELECT fenced FROM robot_commands_v1 WHERE command_id = ?",
                (result.command_id,),
            ).fetchone()
            if row is None:
                raise KeyError(f"命令尚未接受: {result.command_id}")
            state_row = connection.execute(
                "SELECT state FROM robot_commands_v1 WHERE command_id = ?",
                (result.command_id,),
            ).fetchone()
            if state_row is None:
                raise KeyError(f"命令尚未接受: {result.command_id}")
            _validate_transition(CommandState(str(state_row["state"])), result.state, fenced)
            next_fence = bool(row["fenced"]) if fenced is None else bool(fenced)
            if result.state is CommandState.EXECUTION_UNKNOWN:
                next_fence = True
            elif result.state.terminal and fenced is None and not bool(row["fenced"]):
                next_fence = False
            connection.execute(
                "UPDATE robot_commands_v1 SET state = ?, message = ?, output_json = ?, fenced = ? WHERE command_id = ?",
                (
                    result.state.value,
                    result.message,
                    json.dumps(dict(result.output), ensure_ascii=False, sort_keys=True),
                    int(next_fence),
                    result.command_id,
                ),
            )
        return result

    def get(self, command_id: str) -> CommandResult | None:
        """读取命令投影；不存在时返回 ``None``。"""

        with self._lock, self._connect() as connection:
            row = connection.execute(
                "SELECT * FROM robot_commands_v1 WHERE command_id = ?",
                (command_id,),
            ).fetchone()
        return None if row is None else _result_from_row(row)

    def is_fenced(self, command_id: str) -> bool:
        """读取持久 Fence；不存在时返回 False。"""

        with self._lock, self._connect() as connection:
            row = connection.execute(
                "SELECT fenced FROM robot_commands_v1 WHERE command_id = ?",
                (command_id,),
            ).fetchone()
        return bool(row and row["fenced"])

    def fenced_command_ids(self) -> tuple[str, ...]:
        """从 SQLite 返回全部未物理结算命令，跨进程重启保持有效。"""

        with self._lock, self._connect() as connection:
            rows = connection.execute(
                "SELECT command_id FROM robot_commands_v1 WHERE fenced = 1 ORDER BY command_id"
            ).fetchall()
        return tuple(str(row["command_id"]) for row in rows)

    def settle(
        self, result: CommandResult, evidence: PhysicalSettlementEvidence
    ) -> CommandResult:
        """在单一 SQLite 事务中验证 UNKNOWN 并持久解除阻断。"""

        _validate_settlement(result, evidence)
        with self._lock, self._connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            row = connection.execute(
                "SELECT state FROM robot_commands_v1 WHERE command_id = ?",
                (result.command_id,),
            ).fetchone()
            if row is None:
                raise KeyError(f"命令尚未接受: {result.command_id}")
            if CommandState(str(row["state"])) is not CommandState.EXECUTION_UNKNOWN:
                raise CommandRejectedError("只允许显式结算 execution_unknown")
            connection.execute(
                "UPDATE robot_commands_v1 SET state = ?, message = ?, output_json = ?, fenced = 0 WHERE command_id = ?",
                (
                    result.state.value,
                    result.message,
                    json.dumps(dict(result.output), ensure_ascii=False, sort_keys=True),
                    result.command_id,
                ),
            )
        return result


def settle_unknown_as_canceled(
    journal: CommandJournal,
    command_id: str,
    *,
    witness_id: str,
    reason: str,
    source: str,
    confirmed_output: Mapping[str, Any] | None = None,
) -> CommandResult:
    """以操作员确认的物理空闲见证审计式结算一条 UNKNOWN。"""

    normalized_id = str(command_id).strip()
    normalized_witness = str(witness_id).strip()
    normalized_reason = str(reason).strip()
    normalized_source = str(source).strip()
    if not all((normalized_id, normalized_witness, normalized_reason, normalized_source)):
        raise CommandRejectedError("UNKNOWN 人工结算缺少命令、见证、理由或来源")
    existing = journal.get(normalized_id)
    if existing is None:
        raise CommandRejectedError("UNKNOWN 命令不存在")
    audit = existing.output.get("manual_resolution")
    if (
        existing.state is CommandState.CANCELED
        and isinstance(audit, dict)
        and audit.get("witness_id") == normalized_witness
        and audit.get("reason") == normalized_reason
        and audit.get("source") == normalized_source
    ):
        return existing
    if (
        existing.state is not CommandState.EXECUTION_UNKNOWN
        or not journal.is_fenced(normalized_id)
    ):
        raise CommandRejectedError("只允许结算仍被 Fence 阻断的 execution_unknown")
    additions = dict(confirmed_output or {})
    if "manual_resolution" in additions:
        raise CommandRejectedError("物理后端不得覆盖控制面结算审计字段")
    conflicts = {
        key
        for key in additions
        if key in existing.output and existing.output[key] != additions[key]
    }
    if conflicts:
        raise CommandRejectedError(
            f"物理结算输出与既有 UNKNOWN 证据冲突: {sorted(conflicts)}"
        )
    resolved = CommandResult(
        normalized_id,
        CommandState.CANCELED,
        f"操作员确认设备已物理停止: {normalized_reason}",
        {
            **dict(existing.output),
            **additions,
            "manual_resolution": {
                "resolution": "canceled",
                "reason": normalized_reason,
                "witness_id": normalized_witness,
                "source": normalized_source,
                "previous_state": "UNKNOWN",
                "resolved_at_unix": time(),
            },
        },
    )
    return journal.settle(
        resolved,
        PhysicalSettlementEvidence(
            normalized_id,
            CommandState.CANCELED,
            normalized_witness,
            normalized_source,
        ),
    )


def _validate_transition(
    current: CommandState,
    target: CommandState,
    fenced: bool | None,
) -> None:
    """限制账本状态迁移；UNKNOWN 只有显式对账结算才能解除 Fence。"""

    allowed = {
        CommandState.ACCEPTED: {
            CommandState.ACCEPTED,
            CommandState.RUNNING,
            CommandState.REJECTED,
            CommandState.EXECUTION_UNKNOWN,
        },
        CommandState.RUNNING: {
            CommandState.RUNNING,
            CommandState.SUCCEEDED,
            CommandState.FAILED,
            CommandState.CANCELED,
            CommandState.EXECUTION_UNKNOWN,
        },
        CommandState.EXECUTION_UNKNOWN: {CommandState.EXECUTION_UNKNOWN},
    }
    if current.terminal:
        permitted = target is current
    else:
        permitted = target in allowed[current]
    if not permitted:
        raise CommandRejectedError(f"非法命令状态迁移: {current.value} -> {target.value}")


def _validate_settlement(
    result: CommandResult, evidence: PhysicalSettlementEvidence
) -> None:
    """校验结算结果与物理见证一致。"""

    if result.command_id != evidence.command_id:
        raise CommandRejectedError("物理结算见证 command_id 不匹配")
    if result.state is not evidence.terminal_state:
        raise CommandRejectedError("物理结算见证与结果终态不匹配")


def _result_from_row(row: sqlite3.Row) -> CommandResult:
    """把 SQLite 行恢复为公共命令投影。"""

    return CommandResult(
        str(row["command_id"]),
        CommandState(str(row["state"])),
        str(row["message"]),
        json.loads(str(row["output_json"])),
    )
