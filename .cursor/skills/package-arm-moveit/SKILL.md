---
name: package-arm-moveit
description: >-
  Package a 6-DOF robot arm with URDF meshes and MoveIt assets into a
  unilab_robot_template L1 distribution (unilab-arm-<model>). Use when adding
  a new arm model, converting Dobot/vendor ros2_v4 URDF+MoveIt into the
  standard CR7-like package layout, or when the user asks to wrap/import an
  arm into unilab_robot_template.
---

# Package Arm (URDF + MoveIt) → unilab_robot_template

## When to use

User has a normal 6-axis arm (vendor URDF + meshes + optional MoveIt config) and wants an installable L1 package under `packages/unilab-arm-<slug>/`, matching `unilab-arm-cr7` / `unilab-arm-cr5`.

## Hard rules

- **Copy CR7/CR5 as the scaffold**; rename systematically. Do not invent a new layout.
- Arm package owns **model facts only**: joints, limits, FK, URDF/meshes, MoveIt model provider. **No** site points, warehouse sensors, or domain Site bindings (those stay in L3 domain packages).
- MoveIt is **headless**: `build_moveit_model` must not require or launch RViz.
- Qualify all ROS names with `{device_id}_` so two arms can coexist.
- Freeze source URDF with **SHA256**; refuse to build if digest drifts.
- Rewrite mesh URIs from `package://...` to **file://** absolute paths inside the distribution.
- Six revolute joints only; **forbid** rail / `arm_base_joint` in the arm model.
- After packaging: add `pythonpath`, CI `pip install -e`, README row, and `tests/test_<slug>_moveit_model.py`.

## Inputs to collect

1. Model slug (e.g. `cr5`, `cr10`) → package `unilab-arm-<slug>`, import `unilab_arm_<slug>`
2. Source URDF path + mesh directory (all STL/DAE referenced by URDF)
3. Optional: vendor SRDF `disable_collisions` (else copy CR7 adjacency pattern carefully)
4. Source provenance: `repository`, relative `path`, git `exact_ref`, URDF `sha256`

## Workflow

Copy this checklist and complete in order:

```
Progress:
- [ ] 1. Scaffold from unilab-arm-cr7 (or cr5)
- [ ] 2. Vendor assets in
- [ ] 3. Rename identifiers
- [ ] 4. moveit_model.py model-specific fixes
- [ ] 5. model.yaml lock
- [ ] 6. Workspace registration
- [ ] 7. Contract tests pass
```

### 1. Scaffold

Copy `packages/unilab-arm-cr7` → `packages/unilab-arm-<slug>`, rename `src/unilab_arm_cr7` → `src/unilab_arm_<slug>`.

### 2. Vendor assets in

- Place URDF at `models/<slug>_robot.urdf` (exact vendor bytes preferred).
- Place meshes at `models/meshes/<slug>/` — **filenames must match URDF basename** (e.g. CR5 uses `base_link.STL`, CR7 uses `base_link0.STL`).
- Compute SHA256 of the URDF bytes; store lowercase hex.

### 3. Rename identifiers

In all `.py` / `.toml` / `.yaml` under the new package, replace:

| From | To |
|------|-----|
| `unilab-arm-cr7` / `unilab_arm_cr7` | `unilab-arm-<slug>` / `unilab_arm_<slug>` |
| `CR7` / `cr7` | upper/lower slug forms |
| `CR7_JOINT_NAMES` | `<SLUG>_JOINT_NAMES` |
| `dobot-cr7` | vendor-appropriate `model_id` |

Do **not** blindly replace inside unrelated digests. Re-check `_SOURCE_DIGEST` after rename.

Canonical names:

- joints: `<slug>_joint_1` … `<slug>_joint_6`
- links: `device_link`, `<slug>_base`, `<slug>_link_1` … `_6`
- planning group: `<slug>_arm`
- controller: `{device_id}_<slug>_controller`

### 4. `moveit_model.py` fixes

Must implement `build_moveit_model(*, device_id, position=None, rotation=None) -> MoveItModelBundle`:

1. Validate `device_id` (`^[A-Za-z0-9_]+$`)
2. SHA256(URDF) == locked digest
3. All mesh files exist
4. Rename links/joints via maps; set effort/velocity on revolute limits
5. Prefix every name with `{device_id}_`
6. Insert fixed `world` → `device_link` mount (`position` mm → m; `rotation` rad RPY)
7. Append `mock_components/GenericSystem` ros2_control
8. Build SRDF chain `device_link` → `<slug>_link_6` + disable_collisions
9. Return controllers / kinematics / joint_limits dicts; `rviz_required=False`
10. Mesh filenames in the tuple must match files on disk

Reference implementation: `packages/unilab-arm-cr7/src/unilab_arm_cr7/moveit_model.py`.

Also update `kinematics.py` URDF path + joint name prefix; FK reads vendor `joint1`…`joint6` limits/origins.

Adapters (`moveit.py`, plc, tcp_sdk, commissioning) can stay structural clones with renamed joint constants / default `group_name`.

### 5. `models/model.yaml`

```yaml
schema: unilab.robot-model/v1
model_id: <vendor>-<slug>
source:
  repository: <repo>
  path: <path/to/urdf>
  exact_ref: <git-sha>
  sha256: <64-hex>
kinematic_joints:
  - <slug>_joint_1
  # … through _6
forbidden_joints:
  - arm_base_joint
moveit_group: <slug>_arm
rviz_required: false
```

`MODEL_DESCRIPTOR.model_ref` = `package://unilab_arm_<slug>/models/model.yaml`.

### 6. Workspace registration

- Root `pyproject.toml` → `pythonpath`: `packages/unilab-arm-<slug>/src`
- `.github/workflows/quality.yml` → `pip install -e packages/unilab-arm-<slug>`
- `README.md` L1 row for the new arm
- Domain packages (e.g. SZLab) that should use it: add dependency + optional `@device(model={type: package_moveit, provider: "unilab_arm_<slug>.moveit_model:build_moveit_model", source_digest: "..."})` — only when user asks to wire L3

### 7. Tests

Add `tests/test_<slug>_moveit_model.py` mirroring CR7/CR5:

- six movable joints, no rail/rviz
- two `device_id`s → distinct controller names
- digest + 7 meshes on disk; no `package://` left in built URDF
- descriptor joint order/units + zero-pose FK approx values (compute once, lock in test)

Run: `python -m pytest tests/test_<slug>_moveit_model.py tests/test_cr7_moveit_model.py -q`

## Local use without publishing

See [reference.md](reference.md) for PYTHONPATH vs `pip install -e`, OS startup wiring, and provider contract fields.

## Non-goals

- Do not put site/PLC task numbers in the arm package
- Do not merge rail into the arm URDF
- Do not start a separate template process; packages are libraries imported by OS/domain
- Do not enable real hardware MoveIt without an explicit HardwareProfile (mock/sim first)
