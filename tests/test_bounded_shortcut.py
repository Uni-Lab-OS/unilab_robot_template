from unilab_robot_runtime.bounded_shortcut import select_knots


def test_only_qualified_new_edges_are_selected():
    points=list(range(9))
    def valid(a,b,deadline):return b-a<=3
    result=select_knots(points,valid)
    assert result["indices"][0]==0 and result["indices"][-1]==8
    assert all(b-a<=3 for a,b in zip(result["indices"],result["indices"][1:]))
    assert len(result["indices"])<len(points)


def test_expired_edge_keeps_unmodified_native_remainder():
    def timeout(a,b,deadline):raise TimeoutError()
    result=select_knots(list(range(8)),timeout)
    assert result["indices"]==list(range(8))
    assert result["budget_exhausted"] is True
