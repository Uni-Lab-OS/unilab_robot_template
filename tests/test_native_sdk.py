"""Adapter safeguards; these synthetic workers are not vendor SDK evidence."""
import hashlib
import math
from pathlib import Path
import sys

import pytest
from unilab_robot_runtime.hardware_driver import ControllerJointMap,DriverProtocolError,DriverTimeout
from unilab_robot_runtime.native_sdk import NativeSDKAdapter
from unilab_robot_runtime.hardware_driver import create_driver


def make(tmp_path,reply=True):
    worker=tmp_path/'worker.py'
    worker.write_text('import sys,json,time\nfor line in sys.stdin:\n r=json.loads(line)\n'+
      (' print("UNILAB_SDK_REPLY "+json.dumps({"id":r["id"],"ok":True,"result":{"values":[0,0],"units":"deg"}}),flush=True)\n' if reply else ' time.sleep(10)\n'))
    mapping=ControllerJointMap(((1.,0.),(0.,1.)),(0.,0.),(-2.,-2.),(2.,2.),'synthetic')
    return NativeSDKAdapter('elephant280',mapping,python_executable=sys.executable,
        deployment_evidence='fixture',config={'worker_script':str(worker),
        'worker_sha256':hashlib.sha256(worker.read_bytes()).hexdigest()},timeout_s=.25,offline=True)


def test_command_ids_no_replay_and_scope(tmp_path):
    a=make(tmp_path)
    try:
        a.connect('127.0.0.1',1234)
        with pytest.raises(ValueError):a.move_joint((0,0),command_id='')
        receipt=a.move_joint((.1,.2),command_id='one',speed_percent=5)
        assert receipt['state']=='submitted' and not receipt['completed'] and not receipt['hardware_verified']
        with pytest.raises(ValueError):a.move_joint((.1,.2),command_id='one',speed_percent=5)
        assert not a.request_stop('one','fixture')
    finally:a.close()


def test_timeout_and_pinned_worker_quarantine(tmp_path):
    a=make(tmp_path,False)
    with pytest.raises(DriverTimeout):a.connect('127.0.0.1',1234)
    assert a.failed and not a.connected
    with pytest.raises(DriverProtocolError):a.connect('127.0.0.1',1234)
    b=make(tmp_path);b.config['worker_sha256']='0'*64
    with pytest.raises(ValueError):b.connect('127.0.0.1',1234)
    assert b.process is None


def test_invalid_timeout_and_nonloopback(tmp_path):
    a=make(tmp_path)
    with pytest.raises(ValueError):a.connect('192.0.2.1',1234)
    for value in (float('nan'),float('inf'),0,-1):
        with pytest.raises(ValueError):NativeSDKAdapter('x',a.mapping,python_executable=sys.executable,
            deployment_evidence='fixture',config={},timeout_s=value)


def test_model_factory_does_not_infer_rotary_axes(tmp_path):
    class Catalog:
        def module_definition(self,model):
            return {'drivers':{'hardware_candidates':[{'id':'native:igus_cri','adapter':'vendor_native',
                'native_family':'igus_cri','controller_choices':['fixture']}]},'motion_model':{'parameters':{}}}
    a=make(tmp_path)
    options=dict(mapping=a.mapping,controller='fixture',deployment_evidence='fixture',python_executable=sys.executable)
    with pytest.raises(ValueError,match='axis declaration'):
        create_driver(Catalog(),'unknown','native:igus_cri',native_config={'dof':2},**options)
    result=create_driver(Catalog(),'unknown','native:igus_cri',native_config={'dof':2,
        'joint_types':['revolute','revolute'],'joint_type_evidence':'synthetic explicit declaration'},**options)
    assert result.process is None


def test_mixed_axis_units_and_transport_boundary():
    m=ControllerJointMap(((1.,0.),(0.,-1.)),(0.,.2),(-3.,0.),(3.,.5),'fixture frame',
        axis_types=('revolute','prismatic'),native_units=('deg','mm'),unit_evidence='fixture degrees/mm contract')
    assert m.to_native((0.,.15))==pytest.approx((0.,50.))
    assert m.from_native((90.,100.))==pytest.approx((math.pi/2,.1))
    with pytest.raises(ValueError):m.to_degrees((0.,.1))
    with pytest.raises(ValueError):ControllerJointMap(((1.,),),(0.,),(0.,),(1.,),'fixture',axis_types=('prismatic',),native_units=('deg',))
    with pytest.raises(ValueError):ControllerJointMap(((1.,),),(0.,),(0.,),(1.,),'fixture',axis_types=('prismatic',),native_units=('mm',))
