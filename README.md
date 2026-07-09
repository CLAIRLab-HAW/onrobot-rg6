# onrobot-rg6

ROS 2 driver and robot description for the **OnRobot RG6** gripper mounted on a
**Universal Robots** arm (CB3) and controlled through the UR controller's
**tool interface** (no OnRobot Compute Box, URCap **off**).

The driver exposes **all functions the RG6 hardware offers** over that
interface:

| RG6 hardware function | Channel | ROS interface |
|---|---|---|
| Open / close to preset width | Tool-DO0 level (`set_io` pin 16) | `rg6_control/open`, `rg6_control/close` (+ `open_gripper`/`close_gripper` aliases, `std_srvs/Trigger`) |
| Force preset select (low/high) | Tool-DO1 level (`set_io` pin 17) | `rg6_control/set_force_preset` (`std_srvs/SetBool`) |
| **Target width + force** (0–160 mm, 25–120 N) | 17-bit word clocked on Tool-DO0/DO1 (OnRobot URCap protocol) via **URScript injection** | `rg6_control/grip` (`rg6_msgs/Grip`) |
| Width feedback | Tool-AI2 (analog) | `rg6/state.width` [m], joint_states animation |
| Force/current feedback | Tool-AI3 (analog) | `rg6/state.force_raw` |
| **Grip detected** (object grasped) | Tool-DI0 | `rg6/state.grip_detected`, action result `stalled` |
| **Busy/ready** (motion running) | Tool-DI1 (inverted) | `rg6/state.busy`, robust motion-done detection |
| Gripper power | Tool voltage 24 V (`set_io` fun 4) | `rg6_control/set_tool_power` (`std_srvs/SetBool`), auto-on at startup |
| **MoveIt** | — | `rg6_gripper_controller/gripper_cmd` (`control_msgs/GripperCommand` action) + `rg6_moveit_patch` |

Not offered by this hardware generation (documented for completeness): a *stop*
command and direct RS485/Modbus access (CB3 has no Tool Communication
Interface). Depth compensation is an arm motion feature of the URCap, not a
gripper function — do it in MoveIt if needed.

---

## How it works

```
                        this package                        UR driver (ur_robot_driver)
 open/close/preset  ──► rg6_control ──set_io─────────────►  io_and_status_controller ──► Tool-DO0/DO1 ─► RG6
 grip(width,force)  ──► rg6_control ──URScript-Injektion─►  urscript_interface (:30001) ─► 17-bit word ─► RG6
 rg6/state, action  ◄── rg6_control ◄─tool_data/io_states◄─ io_and_status_controller ◄── Tool-AI2/AI3, DI0/DI1
 joint_states       ◄── rg6_joint_state_broadcaster ◄─tool_data
```

- **Level commands** (`open`/`close`): set Tool-DO0 via `set_io`, then wait for
  motion end — primarily via the RG6 **ready signal** (Tool-DI1 from
  `io_states`), falling back to the analog **settle criterion** on Tool-AI2.
  Clamping onto an object (position stable, force high) is reported as
  **success**, and `grip_detected` (Tool-DI0) is included in the response.
- **`grip` (target width/force)**: the RG6's full command set is only reachable
  through the OnRobot 17-bit protocol clocked on Tool-DO0 (clock) / Tool-DO1
  (data, inverted, MSB first): `rg_data = floor(width_mm)*4 +
  floor(force_n/2)*4*161`. The timing is `sync()`-cycle based and cannot be met
  through `set_io` service calls, so `rg6_control` generates the documented
  URScript (community-verified for RG2; RG6 constants parametrized) and
  **injects** it via the `urscript_interface` node.
  **This replaces the running ExternalControl program**; the driver
  automatically calls `io_and_status_controller/resend_robot_program`
  afterwards (`resend_program_after_grip`).
- **`rg6_joint_state_broadcaster`** maps Tool-AI2 to the model's driving joint
  `rg6_finger_joint` (+ 5 follower joints published explicitly — the URDF has
  no `<mimic>`, some parsers/Foxglove choke on it).
- **Simulation** (`rg6_control_sim`): identical ROS surface (services, `grip`,
  action, `rg6/state`) without hardware; publishes the 6 gripper joints itself.
  `sim_object_width_m` emulates an object (close stops there → `grip_detected`).

> **Important:** the UR `io_and_status_controller` must be running (provides
> `set_io`, `tool_data`, `io_states`, `resend_robot_program`), and the RG6 must
> be wired to the UR tool connector. The OnRobot **URCap must stay off**
> (RTDE conflict with `ur_robot_driver`).

---

## Packages

| Package | Type | Contents |
|---|---|---|
| `rg6_control` | `ament_cmake` (C++) | `rg6_control` (driver), `rg6_control_sim`, `rg6_joint_state_broadcaster`, `joint_state_relay`, `joint_state_aggregator`, launch files, `config/rg6_params.yaml`, `scripts/rg6_moveit_patch` |
| `rg6_msgs` | interfaces | `msg/GripperState`, `srv/Grip` |
| `rg6_description` | `ament_cmake` | URDF/Xacro, meshes, Clearpath extras glue |

---

## Build

```bash
# from the workspace root (this folder)
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rg6_description rg6_msgs rg6_control
source install/setup.bash
```

---

## Run

### On real hardware
The nodes use **relative** names — launch them in the **same namespace as the
UR `io_and_status_controller`** (Clearpath: `/a200_0553/manipulators`):

```bash
ros2 launch rg6_control rg6_bringup.launch.py
# args: params_file:=...  start_urscript_interface:=true  robot_ip:=192.168.131.40
```

### Examples

```bash
NS=/a200_0553/manipulators
ros2 service call $NS/rg6_control/close std_srvs/srv/Trigger
ros2 service call $NS/rg6_control/open  std_srvs/srv/Trigger
# target width 80 mm at 60 N (URScript injection, restores ExternalControl):
ros2 service call $NS/rg6_control/grip rg6_msgs/srv/Grip "{width: 0.08, force: 60.0, wait: true}"
# force preset (level mode) high/low:
ros2 service call $NS/rg6_control/set_force_preset std_srvs/srv/SetBool "{data: true}"
# gripper power:
ros2 service call $NS/rg6_control/set_tool_power std_srvs/srv/SetBool "{data: false}"
# state:
ros2 topic echo $NS/rg6/state
```

### Simulation (no hardware)

```bash
ros2 launch rg6_control rg6_control.launch.py gripper_sim:=true
# object in the gripper: -p sim_object_width_m:=0.05 (via params or node args)
```

---

## ROS API (`rg6_control` node)

| Interface | Name (relative) | Type |
|---|---|---|
| Service | `rg6_control/open` / `rg6_control/open_gripper` | `std_srvs/srv/Trigger` |
| Service | `rg6_control/close` / `rg6_control/close_gripper` | `std_srvs/srv/Trigger` |
| Service | `rg6_control/grip` | `rg6_msgs/srv/Grip` |
| Service | `rg6_control/set_force_preset` | `std_srvs/srv/SetBool` |
| Service | `rg6_control/set_tool_power` | `std_srvs/srv/SetBool` |
| Action | `rg6_gripper_controller/gripper_cmd` | `control_msgs/action/GripperCommand` |
| Publisher | `rg6/state` | `rg6_msgs/msg/GripperState` |
| Publisher | `urscript_interface/script_command` | `std_msgs/msg/String` |
| Client | `io_and_status_controller/set_io` | `ur_msgs/srv/SetIO` |
| Client | `io_and_status_controller/resend_robot_program` | `std_srvs/srv/Trigger` |
| Subscriber | `io_and_status_controller/tool_data` | `ur_msgs/msg/ToolDataMsg` |
| Subscriber | `io_and_status_controller/io_states` | `ur_msgs/msg/IOStates` |

`rg6_joint_state_broadcaster`: `tool_data` → `joint_states`
(`rg6_finger_joint` + 5 followers; remap `joint_states` to your bus).

---

## MoveIt

MoveIt talks to the gripper through the standard **GripperCommand** pipeline:

1. **SRDF**: planning group `gripper` (joint `rg6_finger_joint`), end effector
   `rg6` at `arm_0_tool0`, named states `open` (0.0) / `close` (0.6), the 5
   follower joints as `passive_joint`.
2. **moveit.yaml**: `moveit_simple_controller_manager` entry
   `manipulators/rg6_gripper_controller` (type `GripperCommand`, action_ns
   `gripper_cmd`, `max_effort` 60 N) + `joint_limits` for `rg6_finger_joint`
   (TOTG needs acceleration limits).
3. **Action server**: `rg6_control` serves
   `<ns>/rg6_gripper_controller/gripper_cmd`. Goal positions at the endpoints
   (± `action_endpoint_angle_tol`) use the robust level command; intermediate
   widths use the URScript grip (force = goal `max_effort`).
   **Grasp semantics**: clamping on an object returns `stalled: true` +
   **SUCCEEDED** — exactly what MoveIt pick pipelines expect.

Clearpath generates `robot.srdf`/`moveit.yaml` fresh **on every boot** and
knows nothing about OnRobot grippers, so this repo ships an idempotent patch
tool that runs **after generation, before move_group starts**:

```bash
rg6_moveit_patch --setup-path /etc/clearpath   # robot (called by clearpath-custom-setup)
rg6_moveit_patch --setup-path /clearpath       # offboard container (called by entrypoint)
```

It edits `robot.srdf` (marker-framed block, updates in place) and
`manipulators/config/moveit.yaml` (PyYAML round-trip), with `.bak` backups and
atomic writes. The RG6 **collision pairs** need no patching: Clearpath's
`moveit_collision_updater` sees the full URDF including `platform.extras` and
already emits them.

Hooked up in:
- `husky-custom-setup/install-clearpath-custom-setup.sh` → per-boot patcher
  step 4 (`run_rg6_moveit_patch`, before `clearpath-manipulators.service`),
- `husky-offboard/entrypoint.sh` → after the `generate_*` runs.

Usage from MoveIt (named targets or joint goals on group `gripper`):

```python
group = MoveGroupCommander("gripper")          # via moveit_commander / move_group API
group.set_named_target("close"); group.go()    # stops+succeeds on object contact
```

RViz: MotionPlanning panel → Planning Group `gripper` → Goal State
`open`/`close` → Plan & Execute.

> Display note: the URDF intentionally has no `<mimic>` tags, so RViz previews
> only move `rg6_finger_joint`; the real/sim joint states animate all fingers.

### Valid width feedback without a manual warm-up

The RG6 only drives its width analog line (Tool-AI2) **after it has received its
first motion command** since power-on; before that AI2 reads ~0 V. Two mechanisms
keep this from breaking the state and MoveIt:

- **`rg6_control` auto-prime** (`prime_on_ready`, default `true`): on the **rising
  edge of `robot_program_running`** (ExternalControl becomes active — at boot *or*
  when the arm is powered up late) `rg6_control` sets the tool voltage and issues
  one **open** command, so AI2 becomes valid immediately. Runs **once per power-on**
  (reset on `set_tool_power false`) — the ExternalControl restart after a URScript
  `grip` does **not** re-open (would drop a grasped object). Delay after powering:
  `prime_settle_s` (default 1.0 s).
- **`rg6_joint_state_broadcaster` dead-zone gate** (`dead_input_threshold`, default
  0.2 V): AI2 **below** this is treated as “no valid feedback" and the last good
  angle is **held** instead of being mapped to a fake `closed`. Without it, the 0 V
  pre-command reading clamps to `angle_closed` → wrong displayed state **and** a
  poisoned MoveIt start state (planning from the fake value → the gripper can't be
  moved in MoveIt). Must sit below the closed voltage `in_closed` (0.56 V).

---

## Calibration

All values are **live ROS parameters** (`ros2 param set`, persist them in
`config/rg6_params.yaml`).

### Analog width (Tool-AI2 → m)
Watch `io_and_status_controller/tool_data` while opening/closing and set:
`width_in_open`/`width_in_closed` (volts at the ends) —
`width_open_m`/`width_closed_m` default to the RG6 stroke (0.160/0.0).
The same volt endpoints drive the joint animation
(`rg6_joint_state_broadcaster`: `in_open`/`in_closed`, `angle_open`/`angle_closed`).

### Motion detection (fallback without `io_states`)
`force_threshold`, `move_eps`, `settle_eps`, `settle_time_s`,
`motion_timeout_s` — as before; only used when Tool-DI feedback is unavailable.

### 17-bit grip protocol (**verify once against the device!**)
The encoding is community-verified for the **RG2**
(`width*4 + floor(force/2)*4*111`, [python-urx PR#35](https://github.com/SintefManufacturing/python-urx/pull/35)).
For the RG6 the driver parametrizes it with datasheet ranges — validate on the
robot before productive use:

| Parameter | Default | Meaning |
|---|---|---|
| `grip_max_width_mm` | 160 | RG6 stroke |
| `grip_min_force_n` / `grip_max_force_n` | 25 / 120 | RG6 force range |
| `rg_width_steps` | 161 | encoding multiplier (= max width + 1; RG2: 111) |
| `rg_force_divisor` | 2 | force quantization |
| `rg_slave` / `rg_slave_flag` | false / 16384 | second gripper (dual mount) |

Validation procedure (hand on the e-stop, workspace clear):
1. `ros2 service call .../grip "{width: 0.08, force: 40, wait: true}"`
2. compare commanded vs. measured width (`rg6/state.width`, caliper).
3. If the gripper moves to a wrong width, adjust `rg_width_steps` /
   `rg_force_divisor`; if it doesn't react at all, the unit may not accept the
   protocol without prior URCap provisioning — keep using level mode
   (`grip_backend: io` rejects `grip` cleanly).

---

## Using it on the Clearpath Husky (a200-0553)

Unchanged from before: workspace in `system.ros2.workspaces`,
`io_and_status_controller` via `robot.yaml` `ros_parameters`, visual model via
`platform.extras.urdf` (`clearpath_extras.urdf.xacro`), autostart via
`rg6-bringup.service` (now also starts `urscript_interface`). The gripper
publishes joint states on `manipulators/endeffectors/joint_states`; the
`joint_state_relay` mirrors them (RELIABLE) onto `platform/joint_states` for
`robot_state_publisher` + `move_group`, and `joint_state_aggregator` builds the
full `/a200_0553/joint_states` snapshot for recording.

---

## Safety

- Calling the services moves real hardware — keep the workspace clear, hand on
  the e-stop, especially while calibrating.
- `grip` (URScript injection) **interrupts ExternalControl**: never call it
  while an arm trajectory is executing. The driver restores the program via
  `resend_robot_program`; if that fails, run
  `ur_state_manager/ensure_ready`.
- After `set_tool_power {data: false}` the gripper is unpowered; analog/DI
  feedback is meaningless until re-powered.

---

## License

See `src/rg6_description/LICENSE`. `rg6_control`/`rg6_msgs`: BSD-3-Clause.
