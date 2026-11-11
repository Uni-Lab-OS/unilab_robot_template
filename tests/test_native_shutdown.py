"""The native owner must expose crashes and scope the Jazzy loader workaround."""
import json
import os
import subprocess
import sys
from types import SimpleNamespace
import pytest
from unilab_robot_runtime import ros_environment as owner
from unilab_robot_runtime import moveit_move_group as native


def install(prefix, version='2.12.4'):
    share=prefix/'share/moveit_ros_move_group';share.mkdir(parents=True)
    (share/'package.xml').write_text(f'<package><version>{version}</version></package>')
    lib=prefix/'lib/libmoveit_move_group_default_capabilities.so'
    lib.parent.mkdir();lib.touch()
    return lib


@pytest.mark.skipif(not sys.platform.startswith('linux'),reason='Linux ELF lifetime workaround')
def test_known_native_plugin_is_kept_loaded_only_in_child_environment(tmp_path, monkeypatch):
    lib=install(tmp_path)
    monkeypatch.setenv('LD_PRELOAD','/existing.so')
    env=owner.move_group_environment(tmp_path,domain_id=97)
    assert env['LD_PRELOAD'].split()==[str(lib),'/existing.so']
    assert os.environ['LD_PRELOAD']=='/existing.so'
    assert owner.move_group_environment(tmp_path,domain_id=97,base=env)['LD_PRELOAD']==env['LD_PRELOAD']


@pytest.mark.parametrize('version',['2.5.9','2.14.0'])
def test_other_native_versions_are_not_silently_modified(tmp_path,version):
    install(tmp_path,version)
    env=owner.move_group_environment(tmp_path,domain_id=97,base={})
    assert 'LD_PRELOAD' not in env


@pytest.mark.skipif(not sys.platform.startswith('linux'),reason='Linux ELF lifetime workaround')
def test_missing_known_plugin_is_a_configuration_failure(tmp_path):
    install(tmp_path).unlink()
    with pytest.raises(RuntimeError,match='capability'):
        owner.move_group_environment(tmp_path,domain_id=97,base={})


@pytest.mark.parametrize('code,timeout,body_error',[(0,False,False),(-11,False,False),(-9,True,False),(-11,False,True)])
def test_native_exit_is_checked_without_masking_original_failure(tmp_path,monkeypatch,code,timeout,body_error):
    events=[]
    class Process:
        pid=123;returncode=None
        def poll(self):return self.returncode
        def terminate(self):events.append('terminate')
        def kill(self):events.append('kill');self.returncode=-9
        def wait(self,timeout):
            if timeout_flag and 'kill' not in events:raise subprocess.TimeoutExpired('move_group',timeout)
            self.returncode=code;return code
    timeout_flag=timeout
    class Planner:
        def __init__(self,config):pass
        def wait_ready(self):return True
        def close(self):events.append('client_close')
    monkeypatch.setattr(subprocess,'Popen',lambda *a,**kw:Process())
    monkeypatch.setattr(native,'MoveGroupPlanner',Planner)
    monkeypatch.setattr(native,'build_move_group_parameters',lambda *a,**kw:{'ompl':{'arm':{}}})
    monkeypatch.setenv('ROS_DOMAIN_ID','15')
    robot=SimpleNamespace(planning_group='arm',digest='a'*64)
    def run():
        with owner.native_planning_session(robot,None,tmp_path,domain_id=97):
            if body_error:raise ValueError('original planning failure')
    if body_error:
        with pytest.raises(ValueError,match='original planning failure') as caught:run()
        assert any('native-exit.json' in note for note in caught.value.__notes__)
    elif code:
        with pytest.raises(RuntimeError,match='move_group.*shutdown'):run()
    else:run()
    report=json.loads((tmp_path/'native-exit.json').read_text())
    assert report['returncode']==code and report['clean_shutdown']==(code==0)
    assert report['forced_kill']==timeout
    assert events[:2]==['client_close','terminate']
    assert os.environ['ROS_DOMAIN_ID']=='15'
