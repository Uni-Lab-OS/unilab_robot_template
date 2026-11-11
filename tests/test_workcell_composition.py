"""Composition gates exercise actual URDFs and existing endpoint leases."""
import xml.etree.ElementTree as ET
import pytest

from unilab_rail_linear import RailConfiguration, build_configured_model, build_kinematic_model
from unilab_rail_linear.factory import create_simulation_module
from unilab_rail_mounted_arm.composition import Composition
from unilab_rail_mounted_arm.endpoint_guard import EndpointLeaseRegistry
from unilab_robot_contracts import EndpointConflictError


def declaration():
    return dict(schema='unilab.workcell-composition/1', identity='cell', parts={
        'rail':dict(model='rail-a',joints=['rail_joint']),
        'arm':dict(model='arm-a',joints=['arm_joint']),
        'tool':dict(model='installed-tool',joints=['finger_joint'], binding='embedded',
                    geometry=['link:tool','link:finger'],mount_frame='flange',tcp_source='measured-tcp')},
        execution={'plc_program':dict(capabilities=['program','observe'],
                                     control_groups={'plc-program':['rail','arm','tool']})})


def model():
    return '''<robot name="test"><link name="base"/><link name="carriage"/><link name="flange"/>
    <link name="tool"><visual><geometry><box size=".1 .1 .1"/></geometry></visual></link>
    <link name="finger"><visual><geometry><box size=".01 .1 .1"/></geometry></visual></link>
    <joint name="rail_joint" type="prismatic"><parent link="base"/><child link="carriage"/></joint>
    <joint name="arm_joint" type="revolute"><parent link="carriage"/><child link="flange"/></joint>
    <joint name="mount" type="fixed"><parent link="flange"/><child link="tool"/></joint>
    <joint name="finger_joint" type="prismatic"><parent link="tool"/><child link="finger"/></joint></robot>'''


def test_owned_joints_and_embedded_tool_do_not_create_geometry():
    c=Composition(declaration());xml=model();receipt=c.validate_model(xml)
    assert len(receipt['joint_owners'])==3
    assert receipt['tool_geometry']=={'link:tool':'link:tool','link:finger':'link:finger'}
    assert not receipt['external_geometry_pending']
    assert xml==model()


@pytest.mark.parametrize('fault',['joint','geometry','instantiate','control_owner','plc_trajectory'])
def test_ambiguous_or_unsupported_composition_rejected(fault):
    d=declaration()
    if fault=='joint':d['parts']['tool']['joints'].append('arm_joint')
    if fault=='geometry':d['parts']['arm']['geometry']=['link:tool']
    if fault=='instantiate':d['parts']['tool']['instantiate_asset']=True
    if fault=='control_owner':d['execution']['plc_program']['control_groups']['second']=['tool']
    if fault=='plc_trajectory':d['execution']['plc_program']['capabilities'].append('trajectory')
    with pytest.raises(ValueError):Composition(d)


@pytest.mark.parametrize('operation',['trajectory','tool','point','stop'])
def test_plc_cannot_dispatch_unsupported_operations(operation):
    calls=[];binding=Composition(declaration()).execution('plc_program')
    with pytest.raises(ValueError,match='does not support'):
        binding.dispatch(operation,lambda:calls.append('sent'))
    assert calls==[]


def test_program_executes_once_preserving_result_and_error():
    binding=Composition(declaration()).execution('plc_program');calls=[];result={'state':'UNKNOWN'}
    assert binding.dispatch('program',lambda command:(calls.append(command),result)[1],'abc') is result
    def fail():
        calls.append('failed');raise TimeoutError('unconfirmed')
    with pytest.raises(TimeoutError):binding.dispatch('program',fail)
    assert calls==['abc','failed']


def test_unowned_joint_and_missing_tool_geometry_fail_before_use():
    c=Composition(declaration())
    with pytest.raises(ValueError):c.validate_model(model().replace('finger_joint','other_joint'))
    with pytest.raises(ValueError):c.validate_model(model().replace('name="finger"','name="absent"'))


def test_external_geometry_requires_distinct_resolved_bodies():
    d=declaration();d['parts']['tool']['geometry']=['cad:palm','cad:jaw']
    c=Composition(d)
    assert len(c.validate_model(model())['external_geometry_pending'])==2
    with pytest.raises(ValueError):c.validate_model(model(),external_geometry={'cad:palm':'one','cad:jaw':'one'})
    assert not c.validate_model(model(),external_geometry={'cad:palm':'one','cad:jaw':'two'})['external_geometry_pending']


def test_independent_tool_requires_lock_and_exclusive_geometry():
    d=declaration();d['parts']['tool']['binding']='asset'
    with pytest.raises(ValueError,match='pinned'):Composition(d)
    d['parts']['tool']['asset_lock']='installed-tool/assets.lock.json'
    assert Composition(d).validate_model(model())['tool_geometry']


def test_two_configured_rails_move_along_own_axes_and_do_not_leak_legacy_limits():
    configs=[RailConfiguration('x',(0.,1.),.05,(1.,0.,0.),'domain-x'),
             RailConfiguration('y',(-.5,3.),.1,(0.,1.,0.),'domain-y')]
    a,b=[build_configured_model(device_id=i,configuration=c) for i,c in zip(('a','b'),configs)]
    assert set(a.qualified_joint_names).isdisjoint(b.qualified_joint_names)
    for bundle,c in zip((a,b),configs):
        tree=ET.fromstring(bundle.render_urdf);j=tree.find('joint')
        assert tuple(map(float,j.find('axis').get('xyz').split()))==c.direction
        assert float(j.find('limit').get('upper'))==c.travel_m[1]
    module=create_simulation_module(endpoint_ids=frozenset({'sim:y'}),target_data={'targets':{'far':2.8}},
        on_settled=lambda _:None,configuration=configs[1])
    assert module.move_and_settle('one','far').position==2.8
    with pytest.raises(ValueError):create_simulation_module(endpoint_ids=frozenset({'sim:x'}),
        target_data={'targets':{'far':2.8}},on_settled=lambda _:None,configuration=configs[0])
    assert build_kinematic_model(device_id='legacy').source_digest=='9ec7d9833f46c26e02e08f06aecd12495e4ab6753ebd1e47a967f7bf885bf83d'


@pytest.mark.parametrize('bounds,speed,direction', [((0.,0.),.1,(1.,0.,0.)),((0.,1.),0.,(1.,0.,0.)),
    ((0.,1.),.1,(0.,0.,0.)),((0.,float('inf')),.1,(1.,0.,0.))])
def test_invalid_axis_configuration_rejected(bounds,speed,direction):
    with pytest.raises(ValueError):RailConfiguration('bad',bounds,speed,direction,'test')


def test_existing_endpoint_registry_prevents_second_writer():
    registry=EndpointLeaseRegistry();lease=registry.acquire('szlab',frozenset({'plc:robot-program'}))
    with pytest.raises(EndpointConflictError):registry.acquire('standalone-arm',frozenset({'plc:robot-program'}))
    lease.close();registry.acquire('maintenance',frozenset({'plc:robot-program'})).close()


def test_os_assembly_two_instances_with_distinct_installation(tmp_path):
    from types import SimpleNamespace
    import numpy as np
    from unilabos.simulation.scene_robot_model import assemble_robot_model, robot_link_poses
    configs=[RailConfiguration('x',(0.,1.),.05,(1.,0.,0.),'deployment-a'),
             RailConfiguration('y',(-.5,3.),.1,(0.,1.,0.),'deployment-b')]
    bundles={i:build_configured_model(device_id=i,configuration=c) for i,c in zip(('a','b'),configs)}
    # A second independent station is translated and rotated by 90 degrees.
    plan=SimpleNamespace(scene=SimpleNamespace(order=['a','b'],nodes={i:SimpleNamespace(parent=None) for i in bundles}),
      placements={'a':SimpleNamespace(world_mm=(1000,2000,0),world_rotation_row_major=np.eye(3).tolist()),
                  'b':SimpleNamespace(world_mm=(-1000,0,500),world_rotation_row_major=((0,1,0),(-1,0,0),(0,0,1)))})
    xml,_=assemble_robot_model(plan,bundles);(tmp_path/'two-stations.urdf').write_text(xml)
    home=robot_link_poses(xml,{'a_rail_joint':0.,'b_rail_joint':0.})
    moved=robot_link_poses(xml,{'a_rail_joint':.2,'b_rail_joint':.3})
    assert np.allclose(moved['a_rail_carriage'][:3,3],(1.2,2.,0.))
    assert np.allclose(moved['b_rail_carriage'][:3,3],(-1.3,0.,.5))
    for i in bundles: assert np.allclose(moved[i+'_rail_base'],home[i+'_rail_base'])
    only_a=robot_link_poses(xml,{'a_rail_joint':.2,'b_rail_joint':0.})
    assert np.allclose(only_a['b_rail_carriage'],home['b_rail_carriage'])
