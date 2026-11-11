# Collision profiles and saved-path checking

The optional `collision` extra adds a reusable, CPU FCL checker. It consumes the existing `CollisionObject` contract. It does not replace MoveIt planning, OS custody, execution, or simulation time. Install on the tested Linux runtime with `pip install -e "packages/unilab-robot-runtime[collision]"`; `python-fcl` needs a compatible native wheel or a separately provisioned FCL toolchain.

## Data ownership

- An asset owns local-frame collision geometry and its immutable source digest. `exact` retains source triangles and openings; `outer` is a conservative screening representation.
- OS graph / device deployment owns instance identity, placement and current material ownership. `CollisionInstance` applies the OS-resolved rigid `world_from_asset`; copied instances share geometry, not state.
- The device action supplies current moving-body poses, attached payloads, and exact allowed contact pairs. A body that moves during a checked segment must appear in `moving_ids` at every sample.
- The checker caches native geometry, bounds and a phase-specific pair list. It owns no persistent inventory or planner state. Serialize access to each checker instance.

## Minimal assembly

```python
from collections import ChainMap
from unilab_robot_runtime.cad_scene import CadScene
from unilab_robot_runtime.collision_instance import CollisionInstance
from unilab_robot_runtime.refined_collision import RefinedCollisionChecker

# Resolve these paths and expected digests through a verified, locked asset package.
outer = CadScene(outer_path, expected_sha256=outer_digest)
exact = CadScene(exact_path, expected_sha256=exact_digest, lazy=True)
if outer.metadata['source_geometry_sha256'] != exact.metadata['source_geometry_sha256']:
    raise ValueError('Collision profiles describe different sources')

outer_instance = CollisionInstance(outer, 'station.fixture_01', world_from_asset)
exact_instance = CollisionInstance(exact, 'station.fixture_01', world_from_asset)
# Repeat for other instances. Add URDF collision objects under their link IDs.
checker = RefinedCollisionChecker(
    list(outer_instance.values()) + robot_collision_objects,
    ChainMap({o.object_id: o for o in robot_collision_objects}, exact_instance),
)
result = checker.check(
    current_world_poses,
    moving_ids=current_moving_object_ids,
    allowed_pairs=current_explicit_contact_pairs,
)
if result['collision']:
    print(result['pairs'])
```

`world_from_asset` is an explicit metre-valued rigid transform with the asset's declared axis convention converted by the caller. Runtime motion must update object **body** frames, not geometry-local offsets. Resolve instance-prefixed IDs through `CollisionInstance.object_bodies`. Avoid instantiating both a parent asset's complete collider and the same child collider: each physical body has one collision owner in the assembled scene.

Only profiles generated/audited as conservative may be used as `outer`. Matching IDs alone does not establish geometric containment. An outer hit must be confirmed against the exact source; it is not a collision verdict. Do not use a decimated display mesh as an outer profile. The postproc benchmark checks source digests and the full saved-path masks; the asset builder preserves material/contact geometry and emits source membership.

## Pair policy and timing

`FCLSceneChecker.check(poses, moving_ids=..., allowed_pairs=...)` checks moving–scene and moving–moving pairs. Static–static pairs are outside this motion query; an initial assembly audit is a separate operation. Same-link sibling attachments are **not** implicitly exempt. Unknown IDs, self pairs, duplicate exclusions, or missing moving poses raise errors. Explicit static updates take effect immediately. Only mechanically justified SRDF exclusions, bolted interfaces and phase-specific material contact pairs belong in `allowed_pairs`.

Changing the moving set or allowed pairs rebuilds the cached list. Body AABBs and vectorized shape AABBs prune pairs before native FCL. Native poses update only for survivors. Exact meshes are hash-verified at load, lazily decoded/cooked when an outer hit requires them, and reused thereafter. First-use exact decoding is charged to the query; `scene_load_s` is readiness time, not a promise that all exact meshes have been cooked.

Results include `collision`, hit `pairs`, eligible pairs, AABB survivors, native narrow-phase calls, elapsed time and `continuous_collision_proof=False`. The refined checker also reports exact-cache creation and exact narrow-phase calls. No distance/minimum-clearance or continuous-collision claim is made.

`joint_samples(saved_positions, max_step_rad=.002)` preserves internal points and subdivides each edge. Validate saved time, rail state, source geometry and model identity in the caller. Oversized or nonfinite paths fail before yielding a partial result. This fixed joint step is a discrete-checking bound, **not** a bound on Cartesian clearance. Use a separate rail step in metres. Do not construct unplanned connectors between saved segments.

## Postproc replay entry

`postproc-06-asset-workspace/scripts/benchmark_collision_profiles.py` uses the real URDF FK, original robot STL colliders, saved attachments and rail motion. It checks trajectory/index hashes and keeps per-sample collision masks and timings. Its `--exact-profile` option selects conservative screening plus exact confirmation. See the Asset_pipeline performance report for pinned inputs, scoped results and known guide-contact differences.

The new checker is available to device packages and replay verification. Existing MoveIt services and the production planner selection are unchanged; switching live planning or execution requires a separate consumer integration and physical qualification.

For postproc rail replay, pass `--rail-source` with the exact pinned original geometry. The domain adapter reuses `rail_interface_contact` to split seven base shapes and license only two source guide/carriage pairs after endpoint and invariant Y/Z plate-separation proofs. This policy belongs to postproc, not the generic FCL engine. The benchmark records its extra source loading and proof scope separately.
