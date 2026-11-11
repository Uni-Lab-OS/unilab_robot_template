from dataclasses import replace
import pytest
from test_trajectory_artifact import fixture
from unilab_robot_runtime.trajectory_player import TimedMotionPlayer


def make(tmp_path, **overrides):
    req,result,artifact=fixture()
    # 内部点故意偏离端点连线；导轨独立运动，不并入六轴。
    points=tuple(replace(p,q=((.08,)*6 if p.time_s==.2 else p.q),rail_m=p.time_s*.1) for p in result.waypoints)
    artifact.append('move',req,replace(result,waypoints=points,digest=''))
    artifact.events=[dict(id='grasp',kind='grasp',time_s=.2,details={}),
                     dict(id='release',kind='release',time_s=.8,details={})]
    path=tmp_path/'motion.json';artifact.save(path)
    calls=[]
    kwargs=dict(model_digest=req.robot.digest,scene_digest='a'*64,joint_names=req.robot.joint_names,
                current_q=(0.,)*6,current_rail_m=0.,
                apply=lambda q,rail:calls.append(('pose',q,rail)),
                event=lambda e:calls.append(('event',e['kind'])))
    kwargs.update(overrides)
    return TimedMotionPlayer(path,**kwargs),calls


def test_preserves_interior_path_and_separate_rail(tmp_path):
    player,calls=make(tmp_path)
    state=player.advance(.2)
    assert state.q==(.08,)*6
    assert state.rail_m==pytest.approx(.02)
    assert calls[0]==('pose',(.08,)*6,pytest.approx(.02))
    assert calls[1]==('event','grasp')
    assert not player.done
    player.advance(.8)
    assert player.done
    assert [c for c in calls if c[0]=='event']==[('event','grasp'),('event','release')]


def test_large_step_applies_each_event_pose_before_callback(tmp_path):
    player,calls=make(tmp_path);player.advance(2.)
    assert calls[1]==('event','grasp') and calls[3]==('event','release')
    assert calls[2][1]==pytest.approx((.095,)*6)
    assert calls[-1][1]==(.1,)*6
    player.advance(.1)
    assert len([c for c in calls if c[0]=='event'])==2


@pytest.mark.parametrize('override',[{'scene_digest':'b'*64},{'model_digest':'b'*64},
    {'joint_names':('wrong',)*6},{'current_q':(.01,)*6},{'current_rail_m':.01}])
def test_no_wrong_binding_or_unplanned_bridge(tmp_path,override):
    with pytest.raises(ValueError):make(tmp_path,**override)


def test_event_failure_latches_and_cannot_replay(tmp_path):
    def reject(event):raise RuntimeError('attachment readback failed')
    player,calls=make(tmp_path,event=reject)
    with pytest.raises(RuntimeError,match='readback'):player.advance(1.)
    assert not player.done and len(calls)==1
    with pytest.raises(RuntimeError,match='cannot resume'):player.advance(.1)
