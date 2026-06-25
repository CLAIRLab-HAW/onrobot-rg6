# onrobot-rg6

ROS 2 driver and robot description for the **OnRobot RG6** gripper mounted on a
**Universal Robots** arm and controlled through the UR controller's **tool I/O**
(no OnRobot Compute Box required).

The gripper is opened/closed by toggling a UR **tool digital output**, and its
state (width / force) is read back from the UR **tool analog inputs**. All of
this is exposed by the UR driver's `io_and_status_controller`, so this package
is essentially a thin convenience layer on top of `ur_robot_driver`.

---

## How it works

```
  ROS service                 this package                 UR driver (ur_robot_driver)
  open / close  ──►  rg6_control  ──set_io──►  io_and_status_controller ──► UR tool DO16/DO17 ──► RG6
                                          ▲                                  │
                                          └────────── tool_data ◄────────────┘  (analog_input2 = width,
                                                     (force/width feedback)        analog_input3 = force)
```

- **`rg6_control`** offers two `std_srvs/Trigger` services (`open`,
  `close`). On call it sends a `ur_msgs/SetIO` request (function
  `SET_DIGITAL_OUT`, tool pin **16**) and then watches `tool_data` until the
  motion **settles** (position stable for a short time). Settling is used on
  purpose so that *clamping onto an object* (position stops, force stays high)
  is reported as **success**, not as a timeout.
- **`rg6_joint_state_broadcaster`** converts the analog width feedback into a
  `sensor_msgs/JointState` for the gripper's driving joint, so the model can be
  animated in RViz/Foxglove.
- A **simulation** mode (`gripper_sim:=true`) provides the same services without
  hardware, faking the joint motion — handy for testing the description/UI.

> **Important:** the UR `io_and_status_controller` must be running, and the RG6
> must be wired and configured (OnRobot URCap I/O mapping) so that the tool
> digital output triggers a grip and the tool analog inputs report width/force.

---

## Packages

| Package | Type | Contents |
|---|---|---|
| `rg6_control` | `ament_cmake` (C++) | Driver nodes (`rg6_control`, `rg6_joint_state_broadcaster`) + simulation variants + launch file |
| `rg6_description` | `ament_cmake` | URDF/Xacro, meshes and materials of the RG6 |

---

## Prerequisites

- **ROS 2** (developed/tested on Jazzy; other distros likely work).
- A working **`ur_robot_driver`** with an active **`io_and_status_controller`**
  for your arm (provides `set_io` and `tool_data`).
- `ur_msgs` (`sudo apt install ros-$ROS_DISTRO-ur-msgs`) — pulled in by `rosdep`.
- The RG6 physically connected to the UR **tool connector** and its I/O mapping
  configured via the OnRobot URCap.

---

## Build

```bash
# from the workspace root (this folder)
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Run

### On real hardware
The nodes use **relative** topic/service names so they resolve inside whatever
namespace you launch them in. Launch them in the **same namespace as your UR
`io_and_status_controller`**.

```bash
# plain run (replace the namespace with your robot's):
ros2 run rg6_control rg6_control \
  --ros-args -r __ns:=/<your_ur_namespace>

# or via the launch file (add a namespace, see "Clearpath" below):
ros2 launch rg6_control rg6_control.launch.py
```

Open / close (Trigger):

```bash
ros2 service call /<your_ur_namespace>/rg6_control/close std_srvs/srv/Trigger
ros2 service call /<your_ur_namespace>/rg6_control/open  std_srvs/srv/Trigger
```

### Simulation (no hardware)

```bash
ros2 launch rg6_control rg6_control.launch.py gripper_sim:=true
```

---

## ROS API

`rg6_control` node (real mode):

| Interface | Name (relative) | Type |
|---|---|---|
| Service | `rg6_control/open` | `std_srvs/srv/Trigger` |
| Service | `rg6_control/close` | `std_srvs/srv/Trigger` |
| Client | `io_and_status_controller/set_io` | `ur_msgs/srv/SetIO` |
| Subscriber | `io_and_status_controller/tool_data` | `ur_msgs/msg/ToolDataMsg` |

`rg6_joint_state_broadcaster` node (real mode):

| Interface | Name (relative) | Type | Notes |
|---|---|---|---|
| Publisher | `joint_states` | `sensor_msgs/msg/JointState` | publishes joint **`rg6-l_out_joint`** |
| Subscriber | `io_and_status_controller/tool_data` | `ur_msgs/msg/ToolDataMsg` | |

> The broadcaster only makes the model move if the URDF's driving gripper joint
> is **revolute** and named exactly `rg6-l_out_joint` (with the other fingers as
> `mimic` joints). If you embed the gripper as a *fixed* visual, skip the
> broadcaster.

---

## Calibration

Motion thresholds are compile-time constants at the top of
`src/rg6_control/src/rg6_control.cpp` and are **hardware dependent**:

| Constant | Meaning |
|---|---|
| `io_out_fun_pin` (16) | UR tool digital output that triggers open/close |
| `force_threshold` | force (`analog_input3`) above which motion is "started" |
| `move_eps` | min. width change (`analog_input2`) counted as motion |
| `settle_eps` / `settle_time_s` | width considered stable below `settle_eps` for `settle_time_s` |
| `motion_timeout_s` | overall timeout |

Tune them against live data:

```bash
ros2 topic echo /<your_ur_namespace>/io_and_status_controller/tool_data
# watch analog_input2 (width) and analog_input3 (force) while opening/closing
```

---

## Using it on a Clearpath Husky (ROS 2 / `clearpath_config`)

The Clearpath UR namespace is `/<serial>/manipulators` (e.g.
`/a200_0553/manipulators`). Three things to know:

1. **Make this workspace visible to the robot.** Add its `install/setup.bash`
   to `system.ros2.workspaces` in `robot.yaml` (also needed so `package://rg6_description/...`
   meshes resolve, e.g. in Foxglove).
2. **`io_and_status_controller` is not spawned by Clearpath** (only
   `joint_state_broadcaster` + the arm trajectory controller). Spawn it
   yourself, e.g.:
   ```bash
   ros2 run controller_manager spawner io_and_status_controller \
     -c /a200_0553/manipulators/controller_manager \
     --controller-type ur_controllers/GPIOController \
     --param-file <your>/io_and_status_controller.yaml   # tf_prefix: arm_0_
   ```
3. **Run this driver in the manipulators namespace:**
   ```bash
   ros2 run rg6_control rg6_control --ros-args -r __ns:=/a200_0553/manipulators
   ```
   For autostart, wrap it (plus the spawner) in a launch file and reference it
   from `platform.extras.launch` in `robot.yaml`.

For the **visual model** on a Clearpath robot, do **not** include
`rg6_gripper.xacro` directly — its links use the generic name `base_link`, which
collides with the Husky's `base_link`. Instead attach a prefixed copy
(`rg6_*`) to the arm tool frame (`arm_0_tool0`) via `platform.extras.urdf`.

---

## Safety

Calling the services moves real hardware. Keep the workspace around the gripper
clear and keep a hand on the e-stop, especially while calibrating thresholds.

---

## License

See `src/rg6_description/LICENSE`. `rg6_control` is provided as-is.
