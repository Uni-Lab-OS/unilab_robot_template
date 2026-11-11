from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from test_planning_contract import _model
from unilab_robot_contracts.planning import PlanWaypoint
from unilab_robot_runtime.cartesian_line_spline import fit_cartesian_line
from unilab_robot_runtime.trajectory_spline import curve_spline

def model():
    return replace(_model(),urdf_xml='<robot name="r"><link name="world"/><link name="a"/><link name="link6"/><joint name="rail" type="prismatic"><parent link="world"/><child link="a"/><axis xyz="1 0 0"/></joint><joint name="j" type="prismatic"><parent link="a"/><child link="link6"/><axis xyz="0 0 1"/></joint></robot>',
        base_frame="world",joint_names=("j",),joint_limits=((-1.,1.),))

def test_geometric_retiming_keeps_contact_speed_and_rest_boundaries():
    points=[PlanWaypoint((-.1,),.03,0.),PlanWaypoint((.1,),.03,1.)]
    curve=fit_cartesian_line(model(),SimpleNamespace(axis_id="rail"),points,[.3,.05],[.5,.2],[2.,2.],
        translation_limits=(.015,.06,.3),rotation_speed_limit=.1)
    assert curve["duration_s"]>.2/.015
    spline=curve_spline(curve);times=np.linspace(0,curve["source_duration_s"],10001)
    assert np.max(np.abs(spline(times,nu=1)[:,0]))/curve["time_scale"]<=.015
    assert np.max(np.abs(spline(times,nu=2)[:,0]))/curve["time_scale"]**2<=.06
    assert np.allclose(spline([0.,curve["source_duration_s"]])[:,0],[-.1,.1])
    for order in (1,2):
        assert np.max(np.abs(spline([0.,curve["source_duration_s"]],nu=order)))<1e-8

def test_cartesian_retiming_rejects_a_moving_rail():
    points=[PlanWaypoint((-.1,),0.,0.),PlanWaypoint((.1,),.03,1.)]
    with pytest.raises(ValueError,match="locked rail"):
        fit_cartesian_line(model(),SimpleNamespace(axis_id="rail"),points,[.3,.05],[.5,.2],[2.,2.],
            translation_limits=(.015,.06,.3),rotation_speed_limit=.1)
