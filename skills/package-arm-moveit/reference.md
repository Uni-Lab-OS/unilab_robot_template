# package-arm-moveit — reference

## PYTHONPATH vs pip install

For **local Core / monorepo** development, either works if imports resolve:

### A. `pip install -e` (preferred for CI and domain deps)

```bash
python -m pip install -e packages/unilab-robot-contracts
python -m pip install -e packages/unilab-arm-<slug>
# plus rail / rail-mounted-arm / runtime as needed
```

Satisfies SZLab `pyproject.toml` dependency names; entry metadata is correct.

### B. PYTHONPATH only (no pip)

`unilab_robot_template` is **not** a process. Adding every needed `packages/*/src` to `PYTHONPATH` is enough for `import unilab_arm_<slug>` and for `Path(__file__)`-based mesh loading.

Example (bash):

```bash
TEMPLATE=<repo>/unilab_robot_template
export PYTHONPATH="${TEMPLATE}/packages/unilab-robot-contracts/src:\
${TEMPLATE}/packages/unilab-arm-cr7/src:\
${TEMPLATE}/packages/unilab-arm-cr5/src:\
${TEMPLATE}/packages/unilab-rail-linear/src:\
${TEMPLATE}/packages/unilab-rail-mounted-arm/src:\
${TEMPLATE}/packages/unilab-robot-runtime/src:\
${PYTHONPATH}"
```

Then start OS as usual (`Uni-Lab-SZLab/scripts/start-runtime-os.sh`). Extend that script’s `PYTHONPATH` the same way if you skip editable installs.

**Caveats**

| Topic | PYTHONPATH | pip -e |
|-------|------------|--------|
| Import + mesh via `__file__` | OK | OK |
| `pip install szlab` resolving `unilab-arm-cr7` | Not satisfied | Satisfied |
| Console scripts / dist-info | Missing | Present |
| Accidental wrong cwd | Easy to miss a `src` | Harder |

Rule of thumb: day-to-day Core hack → PYTHONPATH is fine; CI / shared venv / “install domain package with deps” → use `pip install -e`.

## OS provider contract

Domain `@device` MoveIt declaration:

```python
model={
    "type": "package_moveit",
    "provider": "unilab_arm_<slug>.moveit_model:build_moveit_model",
    "source_digest": "<64-hex of locked URDF>",
}
```

OS `load_package_moveit_model` imports `provider`, calls it with graph `device_id` / pose, and expects a bundle with:

- `urdf`, `srdf` (str)
- `ros2_controllers`, `moveit_controllers`, `kinematics`, `joint_limits` (dict)
- `source_digest` (str)
- `rviz_required` (bool, must be false for headless)

## Canonical file tree

```text
packages/unilab-arm-<slug>/
├── pyproject.toml
└── src/unilab_arm_<slug>/
    ├── __init__.py
    ├── factory.py              # MODEL_DESCRIPTOR + create_*_backend
    ├── arm_module.py
    ├── standalone_device.py
    ├── kinematics.py
    ├── moveit_model.py         # build_moveit_model
    ├── adapters/
    │   ├── __init__.py
    │   ├── _support.py
    │   ├── moveit.py           # <SLUG>_JOINT_NAMES, MoveItBackend
    │   ├── moveit_client.py
    │   ├── moveit_commissioning.py
    │   ├── plc.py
    │   └── tcp_sdk.py
    └── models/
        ├── model.yaml
        ├── <slug>_robot.urdf
        └── meshes/<slug>/...
```

## Vendor URDF expectations

- Links: `dummy_link`, `base_link`, `Link1`…`Link6` (SolidWorks exporter style used by Dobot CR*)
- Joints: `dummy_joint` (fixed), `joint1`…`joint6` (revolute) with `<limit lower upper>`
- If vendor names differ, adjust `_LINK_NAMES` / `_JOINT_NAMES` maps only; keep **canonical** external names as `<slug>_joint_*`

## Disable collisions

Prefer vendor MoveIt SRDF pairs, mapped to canonical link names. Example CR5-style:

- base↔1 Adjacent; base↔2/4 Never
- 1↔2 Adjacent; 1↔4 Never
- 2↔3, 3↔4, 4↔5, 5↔6 Adjacent; 4↔6 Never
