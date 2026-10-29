"""PLC 整块动作语义与厂商地址/selector 的分层合同。"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class PLCProgramDefinition:
    """不含内存地址或 selector 的 PLC 动作能力声明。"""

    program_ref: str
    pattern_ref: str
    arguments: tuple[str, ...]
    rail_target_ref: str | None


@dataclass(frozen=True, slots=True)
class PLCProgramAdapterBinding:
    """PLCAdapterProfile 中独占的 selector 与参数通道映射。"""

    program_ref: str
    selector: int
    parameter_variables: Mapping[str, str]


@dataclass(frozen=True, slots=True)
class PLCProgramSet:
    """一次激活可用的 PLC whole-block 程序语义集合。"""

    revision: str
    programs: Mapping[str, PLCProgramDefinition]

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> PLCProgramSet:
        """解析程序语义，并拒绝程序号或原始地址泄漏。"""

        if data.get("schema") != "unilab.plc-program-set/v1":
            raise ValueError("PLCProgramSet schema 必须为 unilab.plc-program-set/v1")
        revision = str(data.get("revision", "")).strip()
        raw_programs = data.get("programs")
        if not revision or not isinstance(raw_programs, Mapping):
            raise TypeError("PLCProgramSet 必须包含 revision 与 programs 对象")
        programs: dict[str, PLCProgramDefinition] = {}
        for program_ref, raw in raw_programs.items():
            if not isinstance(raw, Mapping):
                raise TypeError(f"program {program_ref} 必须是对象")
            leaked = {"program_number", "selector", "parameter_variables"}.intersection(raw)
            if leaked:
                raise ValueError(f"PLCProgramSet 泄漏 Adapter 字段: {sorted(leaked)}")
            if str(raw.get("execution_scope")) != "whole_block":
                raise ValueError(f"{program_ref} 必须声明 execution_scope=whole_block")
            arguments = _strings(raw.get("arguments"), f"{program_ref}.arguments")
            rail_target = raw.get("rail_target_ref")
            definition = PLCProgramDefinition(
                str(program_ref),
                str(raw.get("pattern_ref", "")),
                arguments,
                None if rail_target is None else str(rail_target),
            )
            if not definition.program_ref.strip() or not definition.pattern_ref.strip():
                raise ValueError("PLCProgramDefinition identity 与 pattern 不得为空")
            programs[definition.program_ref] = definition
        return cls(revision, programs)


@dataclass(frozen=True, slots=True)
class PLCAdapterProfile:
    """把 PLCProgramSet 绑定到一个控制器的 selector 和内存通道。"""

    programs: Mapping[str, PLCProgramAdapterBinding]

    @classmethod
    def from_mapping(
        cls,
        data: Mapping[str, Any],
        *,
        program_set: PLCProgramSet,
    ) -> PLCAdapterProfile:
        """解析 Adapter 绑定并与程序参数做双向 exact 校验。"""

        if data.get("schema") != "unilab.robot-plc-adapter/v1":
            raise ValueError("PLCAdapterProfile schema 不正确")
        raw_programs = data.get("programs")
        if not isinstance(raw_programs, Mapping):
            raise TypeError("PLCAdapterProfile.programs 必须是对象")
        if set(raw_programs) != set(program_set.programs):
            raise ValueError("PLCAdapterProfile 与 PLCProgramSet 程序引用不完全一致")
        bindings: dict[str, PLCProgramAdapterBinding] = {}
        selectors: set[int] = set()
        for program_ref, raw in raw_programs.items():
            if not isinstance(raw, Mapping):
                raise TypeError(f"adapter program {program_ref} 必须是对象")
            selector = int(raw["selector"])
            if selector in selectors:
                raise ValueError(f"PLCAdapterProfile selector 重复: {selector}")
            selectors.add(selector)
            variables = raw.get("parameter_variables")
            if not isinstance(variables, Mapping):
                raise TypeError(f"{program_ref}.parameter_variables 必须是对象")
            normalized = {str(name): str(value) for name, value in variables.items()}
            expected = set(program_set.programs[str(program_ref)].arguments)
            if set(normalized) != expected:
                raise ValueError(f"{program_ref} 参数通道与 PLCProgramSet 不一致")
            bindings[str(program_ref)] = PLCProgramAdapterBinding(
                str(program_ref),
                selector,
                normalized,
            )
        return cls(bindings)


def _strings(value: Any, field: str) -> tuple[str, ...]:
    """解析无重复非空字符串列表。"""

    if isinstance(value, (str, bytes)):
        raise TypeError(f"{field} 必须是字符串列表")
    try:
        result = tuple(str(item).strip() for item in value)
    except TypeError as exc:
        raise TypeError(f"{field} 必须是字符串列表") from exc
    if any(not item for item in result) or len(result) != len(set(result)):
        raise ValueError(f"{field} 不得包含空值或重复值")
    return result


__all__ = [
    "PLCAdapterProfile",
    "PLCProgramAdapterBinding",
    "PLCProgramDefinition",
    "PLCProgramSet",
]
