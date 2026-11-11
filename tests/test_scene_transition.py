"""Sampled mechanism paths must pass before their candidate endpoint is accepted."""
import copy
import pytest
from unilab_robot_runtime.scene_transition import validate_translation_transition, SceneTransitionRejected

COVERAGE = ["robot_world", "moving_world", "moving_moving", "attachments"]

class State:
    def __init__(self): self.positions = {"x": 0., "z": 0.}
    def capture(self): return copy.deepcopy(self.positions)
    def checkpoint(self): return self.capture()
    def restore(self, saved): self.positions = saved.copy()

def run(state, oracle, **kwargs):
    return validate_translation_transition(state=state, read_positions=state.capture,
        project=state.positions.update, validate=oracle,
        segments=kwargs.pop("segments", [{"z": .002}, {"x": .004}]),
        limits={"x": (0., .01), "z": (0., .01)}, **kwargs)

def test_clearance_is_z_then_x_and_checks_interior():
    state = State(); visited = []
    def validate(snapshot):
        visited.append(snapshot)
        return dict(valid=True, coverage=COVERAGE)
    receipt = run(state, validate)
    assert receipt["status"] == "passed"
    assert state.positions == {"x": .004, "z": .002}
    assert len(visited) == 8
    assert all(v["x"] == 0 for v in visited[:3])
    assert all(v["z"] == .002 for v in visited[3:])
    assert [r["time_s"] for r in receipt["samples"]] == sorted(r["time_s"] for r in receipt["samples"])
    assert receipt["continuous_collision_proof"] is False

def test_interior_collision_rolls_back_all_prior_stages():
    state = State()
    def validate(snapshot):
        return dict(valid=not (.0015 < snapshot["x"] < .0025), coverage=COVERAGE)
    with pytest.raises(SceneTransitionRejected) as error:
        run(state, validate)
    assert state.positions == {"x": 0., "z": 0.}
    receipt = error.value.receipt
    assert receipt["status"] == "failed"
    assert receipt["failed_candidate_state"] == {"x": .002, "z": .002}

def test_robot_only_check_cannot_qualify_moving_obstacles():
    state = State()
    with pytest.raises(SceneTransitionRejected, match="coverage"):
        run(state, lambda _: dict(valid=True, coverage=["robot_world"]))
    assert state.positions == {"x": 0., "z": 0.}

def test_all_stage_targets_checked_before_any_projection():
    state = State()
    with pytest.raises(ValueError, match="limit"):
        run(state, lambda _: pytest.fail("must not query"), segments=[{"z": .002}, {"x": .02}])
    assert state.positions == {"x": 0., "z": 0.}

def test_oracle_failure_restores_candidate():
    state = State()
    def unavailable(_): raise TimeoutError("native collision service stopped")
    with pytest.raises(SceneTransitionRejected, match="stopped"):
        run(state, unavailable)
    assert state.positions == {"x": 0., "z": 0.}

def test_invalid_observed_position_rejected_before_projection():
    state = State(); state.positions["x"] = float("nan")
    with pytest.raises(ValueError, match="initial"):
        run(state, lambda _: pytest.fail("must not query"))

def test_sampling_cannot_be_silently_coarsened():
    with pytest.raises(ValueError, match="sampling"):
        run(State(), lambda _: dict(valid=True, coverage=COVERAGE), max_step_m=.01)

def test_ignored_projection_cannot_succeed_with_a_clear_oracle():
    state = State()
    with pytest.raises(SceneTransitionRejected, match="readback"):
        validate_translation_transition(state=state, read_positions=state.capture,
            project=lambda _: None, validate=lambda _: dict(valid=True, coverage=COVERAGE),
            segments=[{"x": .003}], limits={"x": (0., .01)})
    assert state.positions == {"x": 0., "z": 0.}

def test_source_zero_readback_roundoff_is_recorded_without_clamping():
    state = State(); state.positions["x"] = -3.42e-9
    receipt = run(state, lambda _: dict(valid=True, coverage=COVERAGE))
    assert receipt["initial_state"]["x"] == -3.42e-9
    assert state.positions["x"] == .004

def test_meaningful_initial_limit_violation_still_rejected():
    state = State(); state.positions["x"] = -.001
    with pytest.raises(ValueError, match="initial"):
        run(state, lambda _: pytest.fail("must not query"))
