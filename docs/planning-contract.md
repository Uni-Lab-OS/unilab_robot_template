# 规划合同与 MoveItPlanner

2026-10-27 起，路径规划有一个厂商无关的公共入口：

```python
from unilab_robot_contracts import PlanRequest, PlanResult, MotionPlanner
from unilab_robot_runtime import MoveItPlanner

planner: MotionPlanner = MoveItPlanner()
result: PlanResult = planner.plan(request)
```

`unilab-robot-contracts/src/unilab_robot_contracts/planning.py` 只定义值对象和
端口，不导入 ROS、MoveIt、FCL、OMPL 或 NumPy。`unilab-robot-runtime` 的
`moveit_planner.py` 只在第一次 `plan()` 时导入 MoveItPy；没有 ROS 的环境可以
导入合同、运行严格校验和调用 `validate(scene)`。

## 合同类型

| 类型 | 用途 |
|---|---|
| `RobotModel` | 冻结 URDF/SRDF、规划组、关节顺序、限位和可选 IK 参数 |
| `RailAxis` / `start_rail_m` / `goal_rail_m` | 独立导轨轴与段首/段末坐标；不得并入六轴规划组；`goal_rail_m` 缺省等于起点，绝不回落到 0 |
| `PlanGoal` | 关节目标或绝对位姿目标（二选一） |
| `CollisionWorld` / `CollisionObject` / `CollisionGeometry` | 当前碰撞世界；支持内存/文件三角网格、盒体和圆柱 |
| `AttachmentSpec` | 工具/载荷相对法兰的固定安装关系 |
| `PlanConstraints` | 姿态路径约束、安全姿态、最小间隙和白名单接触对 |
| `PlanRequest` | 一次请求的完整、可摘要输入；`digest` 绑定全部显式字段 |
| `PlanWaypoint` | `q`、`rail_m`、`time_s` 和来源 |
| `PlannerIdentity` | 规划器、实现和真实探测到的引擎版本 |
| `PlanResult` | 状态、路点、碰撞检查计数、安全姿态使用标记、诊断和内容摘要 |
| `MotionPlanner` | 唯一必需方法 `plan(request) -> PlanResult` |
| `SceneValidatingPlanner` | 可选 `validate(scene)`；不声称做过碰撞证明 |

状态只有五个：`success | invalid | collision | unreachable | timeout`。成功
结果必须包含非空、同维度、时间非递减的路点；失败结果可以带部分路点，但调用
方不得把它当可执行轨迹。

严格校验包括：

* 所有数值有限，所有位姿为末行 `[0,0,0,1]` 的正交刚体矩阵；
* 关节维度、关节限位、导轨限位、导轨与六轴的一致性；
* 安全姿态维度、附着对象必须存在于世界内、载荷全部被附着；
* 空成功轨迹拒绝，轨迹摘要被篡改拒绝，结果与请求错配拒绝。

## MoveItPlanner 适配

`MoveItPlanner` 使用 MoveItPy 的 `PlanningComponent`，不创建控制器、不调用
`execute()`。场景对象由 `CollisionWorld` 翻译为 FCL `CollisionObject`；
附着对象使用 `flange_from_body` 局部变换。规划后按
`max_joint_sample_step_rad` 做离散碰撞复采样；这是采样检查，不是连续碰撞
证明。`PlanResult.diagnostics` 记录管线、规划器、段落、墙钟时间和规划失败
原因，不包含 MoveIt 对象。

安全姿态约束通过两段规划实现：`start -> safe_pose -> goal`，结果
`safe_pose_used=True`。最小间隙目前没有原生 MoveItPy 实现，明确 fail closed
为 `invalid`，不会静默忽略。

引擎版本通过 `importlib.metadata`/模块属性探测。未安装的引擎记录为
`unknown`，不伪造版本。

## postproc 兼容层

`postproc_station/simulation/moveit_planning_backend.py` 保留原请求/响应字典
API、工站请求校验、配置和原有调用点；其 `MoveItCADPlanner` 继承
`unilab_robot_runtime.moveit_cad.MoveItCADPlanner`，把工站 hook（水平约束、
缩放、预算、轨迹/端点校验、导轨车身身份）注入共享 FCL/ MoveItPy 引擎。
现存的 53 步 evidence 不要求一次性切换调用方式；新代码应直接使用
`PlanRequest`/`PlanResult`。

## 限制

* 本仓不把导轨做成第七轴；导轨仍然是独立坐标。
* `postproc_moveit_contacts` 的原生 Contact 绑定由 postproc 提供，属于设备
  包依赖，不在公共合同内。
* 真实规划验收需要在装好 MoveIt/ROS 的环境执行；普通 CI 只跑合同、无
  MoveIt 失败路径和 postproc 历史计划回归。


## Stable OS attachment snapshots (2026-10-29)

SceneRequestBuilder.build accepts optional state.attachment_state with schema
unilab.attachment-state/v1, robot_urdf_sha256, bindings and revision.
Each body binding contains link_name and a metre-based column-vector
link_from_body rigid 4x4 matrix. Revision is SHA-256 of the remaining
document encoded as UTF-8 JSON with sorted keys, compact separators and no NaN.

OS KinematicSceneState.attachment_snapshot() exports its authoritative
local matrices. Joint motion does not change this content identity. Finger
motion, grasp and release do. A domain adapter forwards the snapshot; it does
not implement mount caching. restore_attachment_snapshot() restores exact
local matrices after checking model identity and current USD projection.
Candidate checkpoints retain those matrices unchanged.

The runtime validates the schema/digest, complete expected attachment inventory,
links, rigid matrices and world_from_link @ link_from_body against fresh body
poses (absolute matrix tolerance 1e-8, no relative tolerance). This tolerance
validates projection roundoff; **it does not quantize mounts or decide cache
equivalence**. Cache equality remains exact and collision meshes are unchanged.
Legacy states without this field retain world-to-link reconstruction.
The builder deep-copies scene-state provenance into each request so subsequent
caller mutations cannot rewrite an earlier request's saved snapshot.

SceneMotionSampler carries authoritative mounts through joint-motion samples.
For interpolated finger events it omits the endpoint mount snapshot, requiring
reconstruction for that changing state. It never labels an interpolated mount
with an unchanged endpoint revision.

This revision is an OS content identity, separate from the native scene receipt.
After a successful full apply, the runtime writes a unique unilab-scene receipt
to PlanningScene.name and commits it only after native readback verifies both the
receipt and the owned objects/attachments/ACM. Joint-only readback requests
ROBOT_STATE | SCENE_SETTINGS, without attached mesh arrays. A missing/changed
receipt invalidates the loaded cache before the next scene synchronization;
a restarted empty scene must receive its geometry again. Failed acknowledgment
readback cannot commit a reusable cache.

The receipt checks scene continuity at synchronization boundaries; it is not an
atomic transaction spanning later planning/validity RPCs, nor a fingerprint of
every external writer's mutations. Existing exclusive ownership of scene object
IDs remains required. Another owner replacing the global scene name causes
conservative re-verification. Owners must not silently restore a receipt while
changing its owned scene data.
Successful native apply/readback is still required before geometry reuse.
Geometry, world poses, custody, attachment transforms and ACM changes continue
to trigger native synchronization. Full native attachment readback remains at
those transitions; first load and server restarts still require scene creation.
No approximate colliders, reduced collision sampling, or group-only checks are
introduced by this capability.

## Supplemental native assembly queries (2026-10-29)

The FCL backend exempts bodies attached to the same link. Ordinary whole-robot
state validity therefore does not certify the internal geometry of a gripper
and cargo sharing the flange. World-vs-world motion is also outside ordinary
robot-vs-world checks.

`unilab_robot_runtime.world_body_collision` provides two serialized query APIs:

- `check_world_bodies(planner, request, body_ids)`: check selected existing
  world bodies against the parked robot, its tool/cargo and remaining world.
- `check_attached_bodies(planner, request, body_ids=None)`: check existing
  attachments, including pairs sharing a parent link. Omitted IDs select all
  attachments.

Both reuse existing native geometry, temporarily binding the selected body to
a collision-free URDF link unused by other attachments. This avoids FCL's
same-link exemption. No new material custody or executed motion is implied.
Attached queries preserve the original declared touch links; phase ACM
permissions remain explicit. They restore the original binding, shapes, local
poses, ACM and scene receipt before restoring the client's loaded-scene cache.
Failed restoration leaves that cache invalid.

These APIs stop on the first invalid query and return native contact pairs and
penetration depths. They do not provide minimum positive clearance or continuous
collision detection. A failed query may identify an unrelated existing scene
collision, so consumers must inspect the returned pairs. A missing independent
collision-free link is an explicit error.

Ordinary planning does not automatically invoke the supplemental audit. A
qualification consumer must require its receipts for each distinct attachment
assembly, including sampled finger/custody transitions. A positive certificate
can only be reused for internal same-link pairs when geometry, local binding
matrices, touch links and effective pairwise ACM permissions are identical.
World transforms or external collisions must still be checked independently.

Native negative controls in the postproc full audit detected a 100 mm overlap
between two same-flange boxes that ordinary state validity accepted; explicit
ACM contact and separated boxes passed. World-body probing also detected the
parked tool. Transaction tests cover validity failure and failed restoration.


## Alternative legal windings

candidate_chain.legal_winding_candidates enumerates a bounded, cost-ordered
frontier of equivalent common-block joint windings. Every entry/approach/grasp
coordinate must remain inside the model limits. max_candidates bounds the output
without enumerating the full Cartesian product.

The endpoint-nearest winding remains the first candidate, but is not a path
quality certificate: another legal winding can permit a shorter obstacle-free
route. A consumer must plan and collision-check candidates, compare complete
task cost, and replan downstream segments whenever the selected numerical joint
endpoint changes. Display-time angle wrapping and unplanned joins are forbidden.

## Sampled translation-mechanism transitions

unilab_robot_runtime.scene_transition.validate_translation_transition accepts
an offline candidate state, source axis limits, ordered translation target
stages, projection/readback callbacks, and a collision oracle. It samples each
stage at at most 1 mm axis spacing, records intermediate results, and accepts the
endpoint only after all samples pass. Rejection restores the checkpoint and
raises SceneTransitionRejected carrying the failed-state receipt.

The oracle must explicitly cover robot/world, moving/world, moving/moving and
attachments. Robot-only validity cannot qualify a moving obstacle. This API
does not search a route, authorize hardware, prove continuous collision safety,
or qualify nominal timing against actuator dynamics. The caller owns serialized
scene access and native-scene reconciliation after candidate rollback.

Keep station targets, process guards, material followers and approved contact
pairs in the domain package. OS owns inventory, claims and workflow scheduling;
the robot runtime does not acquire a second inventory or scheduler.


### 内容绑定的模型身份

`RobotModel.mesh_sha256` 可显式覆盖 URDF 中的每一个网格 URI。公共
`bind_mesh_identity(model, root=...)` 读取当前文件并核验内容；已绑定模型遇到
字节漂移即拒绝。内容绑定的 `urdf_sha256` 用 `sha256:<内容摘要>` 取代本机
网格路径后计算，因此安装目录或 Windows/Linux 路径改变不影响模型身份。
实际执行仍使用原始 URDF 路径；原始 XML 哈希由运行记录另存。未提供内容
绑定时保持旧摘要行为，不能声称自动具备跨主机可移植身份。

## 直接消费场景校验、后处理和轨迹执行

领域调用 check_scene_state(planner, request, world_body_ids=...)，由公共实现同步场景、检查机器人、明确声明的世界运动体以及全部附件的同 link 接触；任何查询或恢复异常都传播。world_body_ids 为空只表示调用者没有声明世界体运动，不能推断整站世界体两两无碰撞。

validate_smoothed_motion 与 build_shortcut_candidate 分别位于既有 trajectory_quality、bounded_shortcut 模块，领域不维护同名包装文件。shortcut 的速度、加速度和 jerk 限值以及复验的水平/直线误差、Cartesian 速度限值由部署显式传入；姿态约束从原始产物读取，不重建另一组默认约束。复验报告限定为运动段，明确标注未重验事件；输出仍需完整事件验收。

SceneMotionSampler.play 接受 apply 与 observe，向独立执行场景提供与预览相同的全场景采样。开始前核对关节、物料归属和每个运动体位姿；保留全部原始路点、事件边界和事件采样，逐采样检查读回。它是运动学执行合同，不授予物理、控制器插值或硬件资格。规划候选末态不能代替 execute 的反馈，也不能自动结算 OS 库存。
