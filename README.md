# onrobot-rg6

Robot description, MoveIt glue and container mock for the **OnRobot RG6**
gripper on a **Universal Robots** arm (CB3) of the Clearpath Husky `a200-0553`.

**This repository does not drive the real gripper.** On the robot the RG6 is
commanded by **`rg6_grip_bridge`** (onboard, Python, in
[`husky-custom-setup`](../husky-custom-setup/scripts/rg6_grip_bridge.py)) over
**XML-RPC against the OnRobot URCap**. What lives here is everything around
that: the measured model, the MoveIt wiring, the `joint_states` plumbing, and a
mock that offers the same surface without hardware.

## Features

- **The measured rg6_v2 model** (`rg6_description`): URDF/Xacro, visual and
  collision meshes, Clearpath extras glue.
- **One interface on both stages** — `rg6_gripper_controller/gripper_cmd` and
  `rg6/bridge_state`, whether the real bridge or the container mock answers.
- **MoveIt wiring that survives a reboot**: `rg6_moveit_patch` writes the SRDF
  block Clearpath's generator cannot, and *verifies* that the moveit.yaml values
  arrived from `robot.yaml` instead of failing silently.
- **`joint_states` plumbing** for a multi-controller-manager robot: one complete
  aggregate for recording, plus a RELIABLE relay `move_group` actually receives.
- **A container mock** (`rg6_control_sim`) offering exactly the real surface —
  no more, so nobody writes code against something that does not exist.
- **A generated gear table** instead of a formula: the linkage has no closed
  form, and the table is regenerated from the URDF.

## Tech Stack

ROS 2 Jazzy, `ament_cmake` (C++17), `control_msgs`, `sensor_msgs`,
`std_msgs`. No interface package of its own — the state is JSON in a
`std_msgs/String`.

### The canonical interface

Both stages — real robot and offboard container — serve the **same names**, so
a caller never branches on which one it is talking to:

| What | Name (relative to `<ns>`) | Type | Served by |
|---|---|---|---|
| Command | `rg6_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` | `rg6_grip_bridge` (robot) · `rg6_control_sim` (container) |
| State | `rg6/bridge_state` | `std_msgs/String` (flat JSON) | same |
| Model animation | `manipulators/endeffectors/joint_states` | `sensor_msgs/JointState` | same |

`<ns>` is `/a200_0553/manipulators`.

* **`command.position` is the joint value in rad**, not the width: `0.0` open,
  `1.25478` closed. Both receivers convert it through the same gear table (see
  [Width ↔ joint](#width--joint)).
* **`max_effort` is the grip force in N**, clamped to 25…120.
* `rg6/bridge_state` carries `width_m`, `busy`, `grip_detected`, `status`,
  `safety_failed`, `last_command`. It is JSON in a `std_msgs/String` rather
  than a typed message so that a consumer needs no interface package of ours
  built and sourced — `std_msgs` is enough.

> **Read the state topic before claiming anything about the gripper.** A
> `GripperCommand` result acknowledges that the goal was *accepted*; the width
> reached is what `bridge_state` reports afterwards. `rg6_grip_bridge` takes
> **one command at a time** and rejects a second one while moving.

### How it works

```
                  this repository                    husky-custom-setup / UR
  MoveIt, plan_server ──gripper_cmd──▶  rg6_grip_bridge ──XML-RPC──▶ OnRobot URCap ─▶ RG6
                                        rg6/bridge_state  ◀──getters──┘
  RViz / Foxglove     ◀──joint_states── rg6_grip_bridge (5 Hz)

  container, no hardware:
  MoveIt, plan_server ──gripper_cmd──▶  rg6_control_sim  ──▶ same two topics
```

The URCap is an RTDE client itself and occupies `tool_digital_output_mask`, so
`ur_robot_driver` runs on an **input recipe without the
`tool_digital_output*` lines**. ROS therefore cannot set a tool DO at all —
which is why the gripper is commanded through the URCap and not through the
UR tool interface.

### Packages

| Package | Type | Contents |
|---|---|---|
| `rg6_description` | `ament_cmake` | the measured **RG6 v2** model: URDF/Xacro, visual + collision meshes, `config/rg6_v2.yaml`, Clearpath extras glue (`clearpath_extras.urdf.xacro`) |
| `rg6_control` | `ament_cmake` (C++) | `rg6_control_sim` (container mock), `joint_state_relay`, `joint_state_aggregator`, `joint_states.launch.py`, `scripts/rg6_moveit_patch`, `include/rg6_control/finger_kinematics.hpp` |
| `tools/` | — | `derive_finger_kinematics.py`: regenerates the gear table from the URDF |

## Installation

```bash
# from the workspace root (this folder)
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rg6_description rg6_control
source install/setup.bash
```

## Usage

### Container / simulation (no hardware)

`rg6_control_sim` offers the canonical interface above and publishes the
driving joint itself, so the whole MoveIt integration is testable without a
robot. The five follower joints hang off the driver via `<mimic>` in the
`rg6_v2` model and are derived by `robot_state_publisher` and `move_group`.

```bash
ros2 run rg6_control rg6_control_sim --ros-args -r __ns:=/a200_0553/manipulators
# emulate an object: -p sim_object_width_m:=0.05
#   closing stops at that width -> grip_detected=true (as the real bridge reports it)
```

Motion model: the width travels to the target at constant speed. What this
does *not* reproduce are the real RG6 pathologies — a success from this node is
marked as non-hardware truth through the `source` field in `/twin/result`.

### Real robot

Nothing in this repository is started for the gripper. `rg6_grip_bridge` runs
as `clearpath-custom-rg6-grip-bridge`, installed by
[`husky-custom-setup`](../husky-custom-setup/). Command it through the action
above.

### joint_states plumbing (`joint_states.launch.py`)

On a multi-controller-manager Clearpath, wheel, arm and gripper joint states
come from **three** separate sources and are not merged automatically:

* **`joint_state_aggregator`** builds one complete `/a200_0553/joint_states`
  (with velocity + effort) for recording and Foxglove — deliberately *not* in
  the live TF path, where an aggregator would be a single point of failure.
* **`joint_state_relay`** mirrors `manipulators/joint_states` and
  `manipulators/endeffectors/joint_states` back onto
  `platform/joint_states`, which Clearpath's `robot_state_publisher` and
  `move_group` subscribe to. It is a node of its own rather than
  `topic_tools relay` because it publishes with **explicitly RELIABLE** QoS —
  `move_group` subscribes RELIABLE, and a best-effort publisher would never
  reach it (state displayed correctly, planning failing).

## MoveIt

MoveIt talks to the gripper through the standard **GripperCommand** pipeline.
Clearpath regenerates `robot.srdf` and `moveit.yaml` on **every boot** and
knows nothing about OnRobot grippers, so the two halves are supplied
differently:

**`moveit.yaml` comes from `robot.yaml`** — no patching. Under
`manipulators.moveit.ros_parameters.move_group` sit the
`moveit_simple_controller_manager` entry `manipulators/rg6_gripper_controller`
(type `GripperCommand`, action_ns `gripper_cmd`) and the
`robot_description_planning.joint_limits.rg6_finger_joint` block that TOTG
needs. Clearpath's `generate_param` deep-merges them into the generated file.

**`robot.srdf` has no such hook** and is patched:

```bash
rg6_moveit_patch --setup-path /etc/clearpath   # robot (clearpath-custom-setup)
rg6_moveit_patch --setup-path /clearpath       # offboard container (entrypoint)
```

The tool writes a marker-framed block into `robot.srdf` only — atomically, with
a `.bak`, idempotent — containing the planning group `gripper`
(joint `rg6_finger_joint`), the group states `open` (0.0) and `close`
(1.25478), the end effector `rg6` (`parent_link` `arm_0_wrist_3_link`,
`parent_group` `arm_0`) and exactly two `<disable_collisions>` pairs
(`moment_arm` ↔ `finger_tip`, per finger) that Clearpath's
`moveit_collision_updater` does not find on its own.

`parent_group="arm_0"` is required: `RobotModel::buildGroupsInfoEndEffectors`
demands `hasLinkModel(parent_link)` on the parent group, and `arm_0_tool0` is
not a member of the joint-based `arm_0` group.

Afterwards **`verify_moveit_yaml`** checks that the values from `robot.yaml`
actually arrived. If they did not — a stale `robot.yaml` under `/etc/clearpath`
or from `ROBOT_YAML_URL` — it lists every missing key, names the SSOT and exits
**1**, so the case shows up at boot instead of at the first gripper goal. No
caller aborts on it; installer and offboard entrypoint log the code.

Why there is no `robot.yaml` route for the SRDF: `clearpath_config` contains
the string `srdf` nowhere, there is no counterpart to `platform.extras.urdf`,
`robot.srdf.xacro` is generated purely from the arms/grippers/lifts loops, and
`Gripper.MODEL` is a closed enum (`franka_gripper`, `kinova_2f_lite`,
`robotiq_2f_140`, `robotiq_2f_85`) that rejects an RG6 entry with a
`ValueError`.

Using it:

```python
group = MoveGroupCommander("gripper")
group.set_named_target("close"); group.go()    # stops and succeeds on contact
```

RViz: MotionPlanning panel → Planning Group `gripper` → Goal State
`open`/`close` → Plan & Execute.

## Width ↔ joint

The RG6 linkage is not linear, so the mapping between the driving joint
`rg6_finger_joint` [rad] and the clear width between the pad faces [m] is a
**generated table**, not a formula:

* `husky-custom-setup/scripts/rg6_finger_kinematics.json` — joint limits
  `0.0 … 1.25478` rad, maximum interpolation error `4.7e-05` m. The width is
  measured between the two `flex_finger` meshes.
* Regenerate it from the URDF after **every** change to the gripper model:
  `tools/derive_finger_kinematics.py`. The table is generated, not maintained
  by hand — a stale table is a visible file with a date rather than a silent
  drift.
* The upper joint limit is the zero crossing of the width; beyond it the
  fingers pass through each other in the model and the width grows again.
* `include/rg6_control/finger_kinematics.hpp` is the C++ reader used by
  `rg6_control_sim`, so mock and bridge interpolate the same table.

The width the device reports is **larger than the jaws actually are**, by
+3…+5 mm depending on direction and with ~1.7 mm of hysteresis. **The
commanded value is the reference, not the reported one.**

Tool-AI2/AI3 are *not* a width source. The raw voltages remain on
`io_and_status_controller/tool_data` and answer a different question — whether
the tool connector has power at all. AI2 has been measured as mis-calibrated by
up to 17 mm and is not a second opinion.

## Using it on the Clearpath Husky (a200-0553)

Workspace in `system.ros2.workspaces`, `io_and_status_controller` via
`robot.yaml` `ros_parameters`, visual model via `platform.extras.urdf`
(`clearpath_extras.urdf.xacro`). The gripper publishes joint states on
`manipulators/endeffectors/joint_states`.

## Safety

* A `GripperCommand` goal moves real hardware — keep the workspace clear and a
  hand on the e-stop.
* The bridge rejects a second command while one is running. Do not paper over
  that with a retry loop; wait for `busy` to fall **and** for the width to
  settle.
* `busy` still reads `false` for roughly 0.4 s after a command is accepted. A
  loop that only waits for the end of the motion returns immediately in that
  gap — wait for both edges.

## Running Tests

The gripper has no unit suite of its own — what it does in the container is
checked end-to-end against a running stack:

```bash
uv run pytest sdk/plan-bridge/tests/test_offboard_gripper.py    # needs the container
```

## Related

- [husky-custom-setup](../husky-custom-setup/README.md) — `rg6_grip_bridge`,
  the node that drives the real gripper
- [robot-contract](../../contract/robot-contract/README.md) — the profile that
  names the action, the state topic and the gear table
- [husky-offboard](../../deploy/husky-offboard/README.md) — the container that
  builds and runs the mock

## Versioning

[Semantic Versioning](https://semver.org/); see [CHANGELOG.md](CHANGELOG.md).
## License

See `src/rg6_description/LICENSE`. `rg6_control`: BSD-3-Clause.

