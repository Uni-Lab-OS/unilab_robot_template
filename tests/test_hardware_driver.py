"""Protocol-level checks using a separate stdin/stdout process, not live robots."""
import json
import math
from pathlib import Path
import sys

import pytest
from unilab_robot_runtime.hardware_driver import (
    ControllerJointMap, DriverProtocolError, DriverTimeout, LineProcess,
    RoboDKDriverAdapter, installed_driver_command,
)

PEER=Path(__file__).parent/'fixtures/robodk_stdio_peer.py'

def mapping():
    # Includes permutation, sign, offset and a coupled axis, not assumed identity.
    return ControllerJointMap(((0.,-1.),(1.,1.)),(.1,-.2),(-3.,-3.),(3.,3.),'synthetic fixture only')

def adapter(tmp_path,scenario):
    p=tmp_path/'scenario.json';p.write_text(json.dumps(scenario),encoding='utf-8')
    # Windows process startup under concurrent wheel builds can exceed 300ms.
    # The timeout fixture still delays two seconds, beyond this bounded budget.
    return RoboDKDriverAdapter(LineProcess([sys.executable,'-B','-u',str(PEER),str(p)],cwd=tmp_path,timeout_s=1),mapping(),controller='fixture',deployment_evidence='fixture',offline=True)

def connect():
    return {'request':'CONNECT offline.invalid 10000 2','responses':['SMS:Working...','SMS:Ready']}
def joints(values='5.72957795131, -11.4591559026'):
    return {'request':'CJNT','responses':['SMS:Working...','JNTS '+values,'SMS:Ready']}

def test_affine_roundtrip_and_validation():
    m=mapping();q=(.7,-.3);assert m.from_degrees(m.to_degrees(q))==pytest.approx(q)
    with pytest.raises(ValueError):m.to_degrees((9,0))
    with pytest.raises(ValueError):m.to_degrees((math.nan,0))
    with pytest.raises(ValueError):ControllerJointMap(((1,1),(1,1)),(0,0),(-1,-1),(1,1),'singular')

def test_connect_read_and_protocol_rejection(tmp_path):
    a=adapter(tmp_path,[connect(),joints(),{'request':'CJNT','responses':['SMS:Error controller fault']}])
    assert a.connect('offline.invalid',10000)==pytest.approx((0,0),abs=1e-10)
    with pytest.raises(DriverProtocolError,match='fault'):a.read_joints()
    assert a.process.failed
    assert not a.connected
    with pytest.raises(DriverProtocolError):a.read_joints()
    assert not a.request_stop('cmd','no controller evidence')

def test_timeout_quarantines_and_never_retries(tmp_path):
    a=adapter(tmp_path,[connect(),joints(),{'request':'CJNT','delay':2}])
    a.connect('offline.invalid',10000)
    with pytest.raises(DriverTimeout):a.read_joints()
    assert a.process.failed
    assert a.query_command('unknown') is None
    assert a.process.process.poll() is not None

@pytest.mark.parametrize('response',['JNTS 1','JNTS nan,0','JNTS bad,0'])
def test_malformed_feedback_rejected(tmp_path,response):
    a=adapter(tmp_path,[connect(),{'request':'CJNT','responses':[response,'SMS:Ready']}])
    with pytest.raises(DriverProtocolError):a.connect('offline.invalid',10000)

def test_missing_fresh_working_not_completed(tmp_path):
    a=adapter(tmp_path,[{'request':'CONNECT offline.invalid 10000 2','responses':['SMS:Ready']}])
    with pytest.raises(DriverTimeout):a.connect('offline.invalid',10000)

def test_driver_digest_pin_and_path_containment(tmp_path):
    p=tmp_path/'driver.exe';p.write_bytes(b'fixture')
    with pytest.raises(ValueError,match='version'):installed_driver_command({'relative_path':'driver.exe','sha256':'bad'},tmp_path)
    with pytest.raises(ValueError,match='absent'):installed_driver_command({'relative_path':'../outside.exe','sha256':'bad'},tmp_path)


def test_motion_parameters_not_silently_discarded(tmp_path):
    a=adapter(tmp_path,[])
    a.targets={'point':((0,0),(0,0,0,0,0,0))}
    with pytest.raises(ValueError,match='Unsupported motion parameters'):
        a.execute_target('point',{'speed':.1,'resolved_target':{'type':'joint_positions','joint_positions_si':[0,0]}},'x')
    assert a.process.process is None
