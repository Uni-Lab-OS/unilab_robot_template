import math
import pytest
from unilab_robot_runtime.candidate_chain import select_candidate_chain,nearest_legal_winding

def row(q,yaw):
    return dict(rail_m=0.,entry=[q],approach=[q],grasp=[q],yaw_rad=yaw)

def test_pair_custody_prevents_cheap_incompatible_grasp_yaw():
    layers=[[row(0.,0.),row(1.,math.pi)],[row(5.,0.),row(1.1,math.pi)]]
    picked=select_candidate_chain(layers,[0.],0.,
        transition_allowed=lambda i,a,b:i%2==0 or a["yaw_rad"]==b["yaw_rad"])
    assert picked["indices"]==[1,1]
    assert picked["cost"]==pytest.approx(1.1)

def test_winding_adjusts_whole_block_without_exceeding_limits():
    item=dict(rail_m=0.,entry=[6.],approach=[6.1],grasp=[6.2])
    resolved=nearest_legal_winding(item,[0.],[(-math.pi,math.pi)])
    assert resolved["entry"][0]==pytest.approx(6.-2*math.pi)
    assert resolved["grasp"][0]-resolved["entry"][0]==pytest.approx(.2)
    assert item["entry"]==[6.]

def test_alternative_wrist_winding_is_retained_for_native_path_comparison():
    from unilab_robot_runtime.candidate_chain import legal_winding_candidates
    original=row(-math.pi/2,0.)
    candidates=legal_winding_candidates(original,[math.pi/3],[(-2*math.pi,2*math.pi)])
    assert [c["entry"][0] for c in candidates]==pytest.approx([-math.pi/2,3*math.pi/2])
    assert candidates[0]["endpoint_travel_rad"]<candidates[1]["endpoint_travel_rad"]
    assert all(math.sin(c["entry"][0])==pytest.approx(-1.) for c in candidates)
    assert original["entry"]==[-math.pi/2]

def test_winding_alternatives_keep_every_phase_inside_limits():
    from unilab_robot_runtime.candidate_chain import legal_winding_candidates
    block=dict(entry=[-.3],approach=[0.],grasp=[.4])
    candidates=legal_winding_candidates(block,[6.],[(-2*math.pi,2*math.pi)])
    assert len(candidates)==1
    assert candidates[0]["grasp"]==[.4]

def test_multiaxis_winding_frontier_is_bounded_and_cost_ordered():
    from unilab_robot_runtime.candidate_chain import legal_winding_candidates
    block=dict(entry=[.1]*6,approach=[.2]*6,grasp=[.3]*6)
    candidates=legal_winding_candidates(block,[0.]*6,[(-4*math.pi,4*math.pi)]*6,max_candidates=8)
    assert len(candidates)==8
    costs=[c["endpoint_travel_rad"] for c in candidates]
    assert costs==sorted(costs)
    assert len({tuple(c["winding_turns"]) for c in candidates})==8
    for c in candidates:
        assert [b-a for a,b in zip(c["entry"],c["grasp"])]==pytest.approx([.2]*6)
