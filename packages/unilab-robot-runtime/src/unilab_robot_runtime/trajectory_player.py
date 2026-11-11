"""消费已验证的定时轨迹；由调用方的仿真时钟推进，不重做插值或规划。"""
from math import isfinite
from .trajectory_artifact import read_artifact, sample


class TimedMotionPlayer:
    """以独立机械臂/导轨端口执行工件，事件处先写入精确姿态再回调。"""

    def __init__(self, path, *, model_digest, scene_digest, joint_names,
                 current_q, current_rail_m, apply, event):
        value, points = read_artifact(path)
        if (value['robot_model_digest'] != model_digest
                or value['scene_geometry_digest'] != scene_digest
                or tuple(value['joint_names']) != tuple(joint_names)):
            raise ValueError('trajectory binding differs from current model/scene')
        if len(current_q) != len(joint_names) or not all(isfinite(x) for x in (*current_q, current_rail_m)):
            raise ValueError('invalid current state')
        if (max(abs(a-b) for a,b in zip(current_q, points[0].q)) > 1e-6
                or abs(current_rail_m-points[0].rail_m) > 1e-7):
            raise ValueError('unplanned bridge from current state rejected')
        events = value.get('events', [])
        if any(not isfinite(e['time_s']) or not 0 <= e['time_s'] <= points[-1].time_s for e in events):
            raise ValueError('event outside trajectory')
        if any(b['time_s'] < a['time_s'] for a,b in zip(events,events[1:])):
            raise ValueError('events must be ordered')
        self.points = points
        self.events = events
        self.digest = value['digest']
        self.time_s = 0.
        self.event_index = 0
        self.apply = apply
        self.event = event
        self.failed = False

    @property
    def done(self):
        return not self.failed and self.time_s == self.points[-1].time_s and self.event_index == len(self.events)

    def advance(self, duration):
        """允许一个时钟步跨过多个事件，保持各事件真实发生时的姿态。"""
        if self.failed:
            raise RuntimeError('failed trajectory cannot resume')
        if not isfinite(duration) or duration <= 0:
            raise ValueError('duration must be positive and finite')
        target = min(self.time_s + duration, self.points[-1].time_s)
        try:
            while self.event_index < len(self.events) and self.events[self.event_index]['time_s'] <= target:
                event = self.events[self.event_index]
                state = sample(self.points, event['time_s'])
                self.apply(state.q, state.rail_m)
                self.event(event)
                self.event_index += 1
            state = sample(self.points, target)
            self.apply(state.q, state.rail_m)
            self.time_s = target
            return state
        except BaseException:
            self.failed = True
            raise
