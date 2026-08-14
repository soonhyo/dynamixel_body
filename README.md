# dynamixel_body

ROS 1 driver and task-level adapter for the single-axis mannequin yaw joint.
The hardware remains controlled by `dynamixel_general_hw`; this package adds a
small bridge so `hair_task_manager` does not depend on ros_control message
details.

## Hardware computer

First bind the final USB adapter to the stable device name. Stop the body
driver and Dynamixel Wizard, then identify the adapter (prefer the by-id path):

```bash
ls -l /dev/serial/by-id/
rosrun dynamixel_body apply_dynamixel_body_udev.sh \
  --device /dev/serial/by-id/<BODY_ADAPTER>
ls -l /dev/dynamixel_body
```

Use `--dry-run` first to inspect the adapter VID, PID, serial, and generated
rule without changing `/etc`. The installed rule is
`/etc/udev/rules.d/99-dynamixel-body.rules`; remove it with:

```bash
rosrun dynamixel_body apply_dynamixel_body_udev.sh --remove
```

The installer refuses VID/PID-only matching so another identical Dynamixel
adapter cannot be selected accidentally.

After reconnecting the adapter, the kernel tty number may change. This is
expected; verify that the stable link follows it before launching:

```bash
ls -l /dev/dynamixel_body
readlink -f /dev/dynamixel_body
groups
```

Then use the same ROS master as the full pipeline and set `ROS_IP` to the
hardware computer's reachable address. The default launch now uses
`/dev/dynamixel_body`:

```bash
roslaunch dynamixel_body dynamixel_body.launch \
  baud_rate:=1000000 \
  protocol_1_0:=true \
  joint_gui:=false
```

The launch exposes:

- `/hair_task_manager/head_rotation/command` (`std_msgs/Float64`, radians)
- `/hair_task_manager/head_rotation/feedback` (`std_msgs/Float64`, radians)
- `/ros_body/position_joint_trajectory_controller/command`
- `/ros_body/joint_states`

Before automatic use, edit `config/body_config.yaml` and verify:

- `command_sign`: `+1` or `-1` according to physical rotation direction
- `joint_zero_rad`: measured Dynamixel joint angle at logical head angle zero
- `limits`: safe logical head-angle range
- the serial port, motor ID, baud rate, and protocol configuration

The bridge rejects commands when joint feedback is missing/stale and rejects
out-of-range targets by default.

### Manual command test

Check the measured position and controller state before commanding the real
hardware:

```bash
rostopic echo -n 1 /ros_body/joint_states
rosservice call /ros_body/controller_manager/list_controllers
```

The joint-state message must contain `body_joint`, and both
`joint_state_controller` and `position_joint_trajectory_controller` must be
`running`. Send logical head angles to the bridge in radians:

```bash
rostopic pub -1 /hair_task_manager/head_rotation/command \
  std_msgs/Float64 "data: 0.3"
```

Start with a target close to the measured position and make sure the mechanism
is clear. To inspect target tracking:

```bash
rostopic echo /ros_body/position_joint_trajectory_controller/state
```

`desired.positions` should change to the requested target and
`actual.positions` should follow it. A normal command does not necessarily
produce a new line in the `roslaunch` terminal.

For a direct ros_control test that bypasses the bridge:

```bash
rostopic pub -1 /ros_body/position_joint_trajectory_controller/command \
  trajectory_msgs/JointTrajectory \
"{joint_names: ['body_joint'],
  points:
  - positions: [0.3]
    time_from_start: {secs: 1, nsecs: 0}}"
```

Set `joint_gui:=true` for a debounced manual slider that uses this same bridge
and displays the measured logical angle. It intentionally does not bypass the
fresh-feedback and angle-limit checks.

### ROS connection troubleshooting

If publishers and subscribers both appear in `rostopic info` but the
controller's desired position never changes, inspect the actual TCPROS
connections with `rosnode info`. In a healthy graph, the bridge has inbound
connections for the command and joint-state topics, and the controller has an
inbound connection from the bridge.

An old or overloaded ROS master can register both endpoints without delivering
the new-publisher callback. The durable fix is to restart the ROS master and
the graph at a safe maintenance point. If that is not possible, keep the real
command publisher running and reconnect only this control chain:

```bash
# Start the hardware without the required bridge node.
roslaunch dynamixel_body dynamixel_body.launch \
  port_name:=/dev/dynamixel_body \
  baud_rate:=1000000 \
  protocol_1_0:=true \
  launch_head_rotation_bridge:=false

# In another terminal, start and keep the real command publisher running.
# Then start the bridge so it discovers both existing publishers.
rosparam load $(rospack find dynamixel_body)/config/body_config.yaml \
  /head_rotation_dynamixel_bridge
rosrun dynamixel_body head_rotation_dynamixel_bridge.py \
  __name:=head_rotation_dynamixel_bridge

# Reload this controller after the bridge advertises its trajectory topic.
rosservice call /ros_body/controller_manager/switch_controller \
"{start_controllers: [],
  stop_controllers: ['position_joint_trajectory_controller'],
  strictness: 2}"
rosservice call /ros_body/controller_manager/unload_controller \
"{name: 'position_joint_trajectory_controller'}"
rosservice call /ros_body/controller_manager/load_controller \
"{name: 'position_joint_trajectory_controller'}"
rosservice call /ros_body/controller_manager/switch_controller \
"{start_controllers: ['position_joint_trajectory_controller'],
  stop_controllers: [],
  strictness: 2}"
```

This is a temporary recovery procedure. Running `rostopic pub` again creates a
new publisher process, so a broken master may require the connection procedure
again. A long-running pipeline publisher or `joint_gui` avoids replacing the
publisher for every target change.

For AX-12A, `protocol_1_0:=true` is required. If feedback works but the
reported position is outside the physical range or position commands cause
continuous rotation, stop the driver and verify `CW Angle Limit`,
`CCW Angle Limit`, and `Present Position` in Dynamixel Wizard before sending
another command. Position mode normally uses angle limits `0` and `1023`;
limits `0` and `0` select wheel mode.

## Full pipeline computer

```bash
roslaunch hair_task_manager full_pipeline.launch \
  belief_head_rotation:=dynamixel \
  belief_head_rotation_command_topic:=/hair_task_manager/head_rotation/command \
  belief_head_rotation_feedback_topic:=/hair_task_manager/head_rotation/feedback
```

Use the `head_reorient` task in the region selector. `head_pose` is a separate
HIRONX camera-gaze task.
