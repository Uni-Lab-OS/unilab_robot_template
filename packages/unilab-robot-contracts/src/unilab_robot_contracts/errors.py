"""机械臂合同跨适配器共享的结构化异常。"""


class CommandRejectedError(RuntimeError):
    """表示命令在派发前已被确定性拒绝。"""


class DispatchUnknownError(RuntimeError):
    """表示已经可能产生物理副作用，但无法证明结果。"""


class EndpointConflictError(RuntimeError):
    """表示同一物理端点被多个公共 Device 激活。"""
