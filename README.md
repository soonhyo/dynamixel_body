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

Set `joint_gui:=true` for a debounced manual slider that uses this same bridge
and displays the measured logical angle. It intentionally does not bypass the
fresh-feedback and angle-limit checks.

## Full pipeline computer

```bash
roslaunch hair_task_manager full_pipeline.launch \
  belief_head_rotation:=dynamixel \
  belief_head_rotation_command_topic:=/hair_task_manager/head_rotation/command \
  belief_head_rotation_feedback_topic:=/hair_task_manager/head_rotation/feedback
```

Use the `head_reorient` task in the region selector. `head_pose` is a separate
HIRONX camera-gaze task.
