import pytest
from unilab_robot_runtime.urdf_tree import complete_joint_positions

URDF="""<robot name="tool"><joint name="left" type="prismatic"><limit lower="0" upper="0.04"/></joint>
<joint name="right" type="prismatic"><limit lower="0" upper="0.04"/><mimic joint="left" multiplier="1"/></joint></robot>"""


def test_passive_and_mimic_are_kept_out_of_active_arm_state():
    active={'arm':.3,'rail':.1}
    assert complete_joint_positions(URDF,active,{'left':.04})==dict(arm=.3,rail=.1,left=.04,right=.04)
    assert active==dict(arm=.3,rail=.1)

@pytest.mark.parametrize('passive',[{}, {'left':.05}, {'left':float('nan')},
    {'left':.03,'right':.02}, {'arm':.2,'left':.04}, {'left':.03,'unknown':0.}])
def test_missing_invalid_or_overriding_passive_state_rejected(passive):
    with pytest.raises(ValueError):complete_joint_positions(URDF,{'arm':.3},passive)
