"""本地机械臂维护 CLI 的最小公共行为测试。"""

from __future__ import annotations

import io
import json

from test_maintenance_session import FakeCommissioningPort, FakeRuntime
from unilab_robot_contracts import DeploymentMode
from unilab_robot_runtime import build_test_runtime
from unilab_robot_runtime.cli import main


def test_cli_snapshot_uses_exclusive_maintenance_session() -> None:
    """CLI 必须通过 RuntimeBinding 会话读取快照，而不是直接绕过租约。"""

    binding = build_test_runtime(
        FakeRuntime(),
        frozenset({"moveit:cli-test"}),
        owner_id="cli-runtime",
        commissioning_port=FakeCommissioningPort(),
        deployment_mode=DeploymentMode.SIMULATION,
    )
    output = io.StringIO()

    exit_code = main(
        ["--owner", "operator-a", "snapshot"],
        runtime_factory=lambda: binding,
        output=output,
    )

    payload = json.loads(output.getvalue())
    assert exit_code == 0
    assert payload["online"] is True
    assert payload["idle"] is True
