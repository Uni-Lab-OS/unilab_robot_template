"""公共规划阶段计时；嵌套阶段为包含子阶段的墙钟时间，禁止相加冒充总耗时。"""
from contextlib import contextmanager
from functools import wraps
from time import perf_counter


class PlanningTiming:
    def __init__(self):
        self._rows = {}

    @contextmanager
    def measure(self, stage):
        started = perf_counter()
        failed = False
        try:
            yield
        except BaseException:
            failed = True
            raise
        finally:
            row = self._rows.setdefault(stage, {"calls": 0, "wall_s": 0.0, "failures": 0})
            row["calls"] += 1
            row["wall_s"] += perf_counter() - started
            row["failures"] += int(failed)

    def snapshot(self):
        return {name: dict(row) for name, row in self._rows.items()}

    def since(self, before):
        return {name: {key: value - before.get(name, {}).get(key, 0)
                       for key, value in row.items()}
                for name, row in self._rows.items()
                if row["calls"] != before.get(name, {}).get("calls", 0)}


def timed_stage(stage):
    """装饰同一规划器的串行调用；RPC 包含序列化、通信与服务执行。"""
    def decorate(function):
        @wraps(function)
        def measured(self, *args, **kwargs):
            timing = getattr(self, "_timing", None)
            if timing is None:
                return function(self, *args, **kwargs)
            label = "rpc." + args[0] if stage == "rpc" else stage
            with timing.measure(label):
                return function(self, *args, **kwargs)
        return measured
    return decorate
