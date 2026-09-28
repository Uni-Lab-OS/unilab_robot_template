"""单写者关节段参考执行器，使用SI输入与实际反馈判定完成。

从冻结CR12/CTAG实验迁移；取消保持测量位置，不代表保护停机。
不启动时钟、仿真世界或线程，不充当PLC程序、ROS动作服务器或厂家SDK。
"""
from __future__ import annotations
import math


def vector(v, count):
    if not isinstance(v, (list, tuple)) or len(v) != count or any(type(x) not in (int,float) or not math.isfinite(x) for x in v):
        raise ValueError('invalid vector')
    return list(v)


class JointSegmentExecutor:
    def __init__(self, names, limits, revision, session, velocity=.35, acceleration=1.0,
                 position_tolerance=.005, velocity_tolerance=.015, settle_s=.15, timeout_s=3.):
        self.names = list(names); self.limits = limits; self.revision = revision; self.session = session
        self.velocity = velocity; self.acceleration = acceleration
        self.ptol = position_tolerance; self.vtol = velocity_tolerance; self.settle_s = settle_s; self.timeout_s = timeout_s
        self.hold = None; self.observation = None; self.active = None; self.seen = set(); self.events = []

    def observe(self, q, dq, sim_s, sequence):
        q = vector(q, len(self.names)); dq = vector(dq, len(self.names))
        if type(sim_s) not in (int,float) or not math.isfinite(sim_s) or sim_s < 0 or type(sequence) is not int or sequence < 0:
            raise ValueError('invalid observation clock')
        if self.observation and (sim_s <= self.observation['sim_s'] or sequence <= self.observation['sequence']):
            raise ValueError('stale observation')
        self.observation = dict(q=q, dq=dq, sim_s=sim_s, sequence=sequence)
        if self.hold is None:
            self.hold = q[:]
        a = self.active
        if a is None: return
        eligible = a['state'] == 'CANCELING' or sim_s >= a['start_s']+a['duration_s']
        target = a['hold'] if a['state'] == 'CANCELING' else a['target']
        settled = max(abs(x-y) for x,y in zip(q,target)) <= self.ptol and max(abs(x) for x in dq) <= self.vtol
        if eligible and settled:
            if a['settled_since'] is None: a['settled_since'] = sim_s
            if sim_s-a['settled_since'] >= self.settle_s:
                self._finish('CANCELED' if a['state'] == 'CANCELING' else 'SUCCEEDED')
        else: a['settled_since'] = None
        if self.active:
            deadline = a['cancel_s']+self.timeout_s if a['state'] == 'CANCELING' else a['start_s']+a['duration_s']+self.timeout_s
            if sim_s > deadline: self._finish('ABORTED', 'feedback_timeout')

    def submit(self, command):
        if not isinstance(command, dict):raise ValueError('command must be an object')
        if self.active: raise ValueError('busy')
        o = self.observation
        if o is None: raise ValueError('no feedback')
        if command.get('model_revision') != self.revision or command.get('session') != self.session:
            raise ValueError('stale model or session')
        if command.get('joint_names') != self.names: raise ValueError('joint ordering')
        if type(command.get('observation_sequence')) is not int or command.get('observation_sequence') != o['sequence']: raise ValueError('stale target observation')
        cid = command.get('command_id')
        if not isinstance(cid, str) or not cid or len(cid)>128 or cid in self.seen: raise ValueError('invalid or reused command ID')
        target = vector(command.get('positions'),len(self.names))
        duration = command.get('duration_s')
        if type(duration) not in (int,float) or not math.isfinite(duration) or duration <= 0: raise ValueError('invalid duration')
        for q,t,lim in zip(o['q'],target,self.limits):
            if not lim['lower'] <= t <= lim['upper']: raise ValueError('joint limit')
            if 1.875*abs(t-q)/duration > min(self.velocity,lim['velocity'])+1e-12: raise ValueError('velocity budget')
            if 5.773502692*abs(t-q)/duration**2 > self.acceleration+1e-12: raise ValueError('acceleration budget')
        if max(abs(x) for x in o['dq']) > self.vtol: raise ValueError('start is moving')
        self.seen.add(cid)
        self.active = dict(command_id=cid, target=target, start=o['q'][:], start_s=o['sim_s'], duration_s=duration,
                           state='EXECUTING', settled_since=None)
        self.events.append(dict(command_id=cid,state='ACCEPTED',sim_s=o['sim_s']))

    def setpoint(self, sim_s):
        if self.observation is None: raise ValueError('no feedback')
        a = self.active
        if a is None: return self.hold[:] if self.hold is not None else self.observation['q'][:]
        if a['state'] == 'CANCELING': return a['hold'][:]
        u = min(1.,max(0.,(sim_s-a['start_s'])/a['duration_s']))
        blend = 10*u**3-15*u**4+6*u**5
        return [q+(t-q)*blend for q,t in zip(a['start'],a['target'])]

    def cancel(self, command_id):
        if self.active is None or self.active['command_id'] != command_id: raise ValueError('unknown active command')
        a = self.active
        if a['state'] == 'CANCELING': return
        a.update(state='CANCELING',hold=self.observation['q'][:],cancel_s=self.observation['sim_s'],settled_since=None)
        self.events.append(dict(command_id=command_id,state='CANCELING',sim_s=self.observation['sim_s'],hold=a['hold'][:]))

    def _finish(self, state, reason=None):
        self.hold = self.active['target'][:] if state == 'SUCCEEDED' else (self.active['hold'][:] if state == 'CANCELED' else self.observation['q'][:])
        self.events.append(dict(command_id=self.active['command_id'],state=state,reason=reason,
                                sim_s=self.observation['sim_s'],actual_positions=self.observation['q'][:]))
        self.active = None
