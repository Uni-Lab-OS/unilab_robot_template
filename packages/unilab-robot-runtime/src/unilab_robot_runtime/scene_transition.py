"""Validate a staged translation-axis scene transition before accepting its endpoint.

Projection is an offline candidate operation. The supplied oracle must cover the
moving mechanism, its payloads, other world bodies and the parked robot. This
does not execute hardware and does not assert continuous collision detection.
"""
import copy
import math

class SceneTransitionRejected(RuntimeError):
    def __init__(self, message, receipt):
        super().__init__(message)
        self.receipt = receipt

def validate_translation_transition(*, state, read_positions, project, validate,
                                    segments, limits, max_step_m=.001,
                                    velocity_m_s=.03, readback_tolerance_m=2e-6):
    if not callable(validate):
        raise ValueError("A full-scene collision validator is required")
    if not 0 < max_step_m <= .001 or not math.isfinite(velocity_m_s) or velocity_m_s <= 0:
        raise ValueError("Invalid translation sampling or velocity")
    if not math.isfinite(readback_tolerance_m) or not 0 <= readback_tolerance_m <= max_step_m:
        raise ValueError("Invalid translation readback tolerance")
    if not segments:
        raise ValueError("A transition needs explicit stages")
    stages = copy.deepcopy(segments)
    for targets in stages:
        if not targets:
            raise ValueError("Empty translation stage")
        for name, value in targets.items():
            if name not in limits or not math.isfinite(value) or not limits[name][0] <= value <= limits[name][1]:
                raise ValueError("Unknown axis or source limit exceeded: " + name)
    observed = read_positions()
    for name in {n for stage in stages for n in stage}:
        bounds = limits[name]
        if (len(bounds) != 2 or not all(math.isfinite(v) for v in bounds)
                or bounds[0] > bounds[1] or name not in observed
                or not math.isfinite(observed[name])
                or not bounds[0]-readback_tolerance_m <= observed[name] <= bounds[1]+readback_tolerance_m):
            raise ValueError("Invalid initial state or source limits: " + name)
    checkpoint = state.checkpoint()
    initial = state.capture()
    receipt = dict(schema="unilab.checked-scene-transition/v1", status="planning",
        initial_state=initial, segments=[], samples=[], max_joint_step_m=max_step_m,
        continuous_collision_proof=False, hardware=False, physics=False,
        readback_tolerance_m=readback_tolerance_m,
        timing_qualification="nominal constant-rate samples; actuator dynamics not qualified")
    elapsed = 0.
    required = {"robot_world", "moving_world", "moving_moving", "attachments"}
    try:
        for index, targets in enumerate(stages):
            start = {name: float(read_positions()[name]) for name in targets}
            distance = max(abs(targets[n]-start[n]) for n in targets)
            count = max(1, math.ceil(distance/max_step_m))
            duration = max(.02, distance/velocity_m_s)
            receipt["segments"].append(dict(index=index,start=start,target=targets,
                                            start_time_s=elapsed,duration_s=duration))
            for sample_index in range(count+1):
                fraction = sample_index/count
                positions = {n: start[n]+(targets[n]-start[n])*fraction for n in targets}
                project(positions)
                observed = read_positions()
                if any(n not in observed or not math.isfinite(observed[n])
                       or abs(observed[n]-v) > readback_tolerance_m for n,v in positions.items()):
                    raise ValueError("Projected translation readback differs from candidate target")
                snapshot = state.capture()
                result = validate(snapshot)
                row = dict(segment=index,time_s=elapsed+duration*fraction,
                           positions=dict(read_positions()),validation=copy.deepcopy(result))
                receipt["samples"].append(row)
                if not isinstance(result,dict) or result.get("valid") is not True:
                    raise ValueError("Scene transition collision or missing validity")
                if not required.issubset(set(result.get("coverage",()))):
                    raise ValueError("Scene validator omitted required collision coverage")
            elapsed += duration
        receipt.update(status="passed",final_state=state.capture(),duration_s=elapsed)
        return receipt
    except Exception as error:
        receipt.update(status="failed",error=str(error),failed_candidate_state=state.capture())
        state.restore(checkpoint)
        raise SceneTransitionRejected(str(error),receipt) from error
