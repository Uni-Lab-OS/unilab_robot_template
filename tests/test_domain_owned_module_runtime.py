from __future__ import annotations

import sys
import types

import pytest
from unilab_robot_runtime.factory import _module_impl


class _ModuleRef:
    def __init__(self, *, python_package: str, version: str = "0.1.0") -> None:
        self.distribution = python_package.replace("_", "-")
        self.version = version
        self.python_package = python_package
        self.endpoint_ids = frozenset()


@pytest.fixture
def domain_arm_module() -> types.ModuleType:
    module = types.ModuleType("my_lab_arm")
    module.MODULE_API_VERSION = 1
    module.MODULE_KIND = "arm"
    module.__version__ = "0.1.0"
    module.MODEL_DESCRIPTOR = object()
    sys.modules["my_lab_arm"] = module
    yield module
    sys.modules.pop("my_lab_arm", None)


@pytest.fixture
def domain_rail_module() -> types.ModuleType:
    module = types.ModuleType("my_lab_rail")
    module.MODULE_API_VERSION = 1
    module.MODULE_KIND = "rail"
    module.__version__ = "0.1.0"
    module.MODEL_DESCRIPTOR = object()
    sys.modules["my_lab_rail"] = module
    yield module
    sys.modules.pop("my_lab_rail", None)


def test_runtime_loads_domain_owned_arm_module(domain_arm_module: types.ModuleType) -> None:
    loaded = _module_impl(_ModuleRef(python_package="my_lab_arm"), kind="arm")
    assert loaded is domain_arm_module


def test_runtime_loads_domain_owned_rail_module(domain_rail_module: types.ModuleType) -> None:
    loaded = _module_impl(_ModuleRef(python_package="my_lab_rail"), kind="rail")
    assert loaded is domain_rail_module


def test_runtime_rejects_wrong_module_kind(domain_arm_module: types.ModuleType) -> None:
    del domain_arm_module
    with pytest.raises(ValueError, match="不是 rail 模块"):
        _module_impl(_ModuleRef(python_package="my_lab_arm"), kind="rail")
