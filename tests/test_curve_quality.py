import pytest
from unilab_robot_runtime.trajectory_quality import curve_joint_travel,measure_segment

def test_variation_counts_internal_reversal_even_when_endpoints_match():
    # q(t)=-t(1-t), q(0)=q(1)=0; excursion to -0.25 and back.
    curve=dict(knots=[0,0,0,1,1,1],coefficients=[[0,2],[-.5,2],[0,2]],degree=2,source_duration_s=1.,duration_s=2.,time_scale=2.)
    assert curve_joint_travel(curve)==pytest.approx([.5,0.])

def test_native_metric_api_cannot_silently_measure_old_path_for_new_curve():
    with pytest.raises(ValueError,match="Native-only"):
        measure_segment(None,None,{"execution_curve":{}})


@pytest.mark.parametrize("limits", [
    (float("nan"), .001, (.15, .5)),
    (.002, 0., (.15, .5)),
    (.002, .001, (-1., .5)),
    (.002, .001, (.15, float("inf"))),
])
def test_invalid_process_limits_rejected_before_reading_or_mutating_scene(limits, tmp_path):
    from unilab_robot_runtime.trajectory_quality import validate_smoothed_motion
    tilt, line, speed = limits
    output = tmp_path / "no-artifact"
    with pytest.raises(ValueError, match="Explicit positive"):
        validate_smoothed_motion(None, None, tmp_path/"missing.json", output,
            linear_segments=(), max_payload_tilt_rad=tilt, max_line_error_m=line,
            cartesian_velocity_limits=speed)
    assert not output.exists()
