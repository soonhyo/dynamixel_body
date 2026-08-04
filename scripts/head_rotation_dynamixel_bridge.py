#!/usr/bin/env python3
"""Bridge the hair pipeline head-angle API to dynamixel_general_hw.

The task pipeline deliberately exposes a hardware-independent pair of
``std_msgs/Float64`` topics.  ``dynamixel_general_hw`` instead accepts a
``trajectory_msgs/JointTrajectory`` and reports ``sensor_msgs/JointState``.
This node is the single owner of that conversion, including the installation
specific zero and direction convention.
"""

import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def head_to_joint(angle_rad, command_sign, joint_zero_rad):
    """Convert the logical head angle to the controlled Dynamixel joint."""
    return float(joint_zero_rad) + float(command_sign) * float(angle_rad)


def joint_to_head(joint_rad, command_sign, joint_zero_rad):
    """Convert measured Dynamixel joint position to the logical head angle."""
    sign = float(command_sign)
    if abs(sign) < 1e-12:
        raise ValueError('command_sign must be nonzero')
    return (float(joint_rad) - float(joint_zero_rad)) / sign


def joint_position(message, joint_name):
    """Return ``joint_name`` from a JointState, or ``None`` if unavailable."""
    try:
        index = list(message.name).index(str(joint_name))
    except ValueError:
        return None
    if index >= len(message.position):
        return None
    value = float(message.position[index])
    return value if math.isfinite(value) else None


class HeadRotationDynamixelBridge:
    def __init__(self):
        self.joint_name = str(rospy.get_param(
            '~joint_name', 'body_joint')).strip()
        self.command_sign = float(rospy.get_param('~command_sign', 1.0))
        self.joint_zero_rad = float(rospy.get_param('~joint_zero_rad', 0.0))
        self.minimum_head_rad = float(rospy.get_param(
            '~limits/min_head_rad', -math.pi))
        self.maximum_head_rad = float(rospy.get_param(
            '~limits/max_head_rad', math.pi))
        self.trajectory_duration = float(rospy.get_param(
            '~trajectory_duration_s', 1.0))
        self.feedback_timeout = float(rospy.get_param(
            '~feedback_timeout_s', 1.0))
        self.require_fresh_feedback = bool(rospy.get_param(
            '~require_fresh_feedback_before_command', True))
        self.clamp_commands = bool(rospy.get_param(
            '~clamp_commands', False))

        if not self.joint_name:
            raise ValueError('joint_name must not be empty')
        if not math.isfinite(self.command_sign) or abs(self.command_sign) < 1e-12:
            raise ValueError('command_sign must be finite and nonzero')
        if not math.isfinite(self.joint_zero_rad):
            raise ValueError('joint_zero_rad must be finite')
        if not (math.isfinite(self.minimum_head_rad)
                and math.isfinite(self.maximum_head_rad)
                and self.minimum_head_rad < self.maximum_head_rad):
            raise ValueError('head limits must be finite and min < max')
        if not math.isfinite(self.trajectory_duration) or self.trajectory_duration <= 0.0:
            raise ValueError('trajectory_duration_s must be positive')
        if not math.isfinite(self.feedback_timeout) or self.feedback_timeout <= 0.0:
            raise ValueError('feedback_timeout_s must be positive')

        command_topic = str(rospy.get_param(
            '~topics/command',
            '/hair_task_manager/head_rotation/command')).strip()
        feedback_topic = str(rospy.get_param(
            '~topics/feedback',
            '/hair_task_manager/head_rotation/feedback')).strip()
        trajectory_topic = str(rospy.get_param(
            '~topics/trajectory_command',
            '/ros_body/position_joint_trajectory_controller/command')).strip()
        joint_state_topic = str(rospy.get_param(
            '~topics/joint_states', '/ros_body/joint_states')).strip()
        if not all((command_topic, feedback_topic, trajectory_topic,
                    joint_state_topic)):
            raise ValueError('all topic parameters must be nonempty')

        self._last_feedback_receipt = None
        self._last_joint_position = None
        self._trajectory_pub = rospy.Publisher(
            trajectory_topic, JointTrajectory, queue_size=1)
        self._feedback_pub = rospy.Publisher(
            feedback_topic, Float64, queue_size=1, latch=True)
        self._joint_sub = rospy.Subscriber(
            joint_state_topic, JointState, self._joint_state_cb, queue_size=1)
        self._command_sub = rospy.Subscriber(
            command_topic, Float64, self._command_cb, queue_size=1)

        rospy.loginfo(
            '[HeadRotationBridge] ready joint=%s command=%s feedback=%s '
            'trajectory=%s states=%s sign=%+.3f zero=%.4f limits=[%.3f, %.3f]',
            self.joint_name, command_topic, feedback_topic, trajectory_topic,
            joint_state_topic, self.command_sign, self.joint_zero_rad,
            self.minimum_head_rad, self.maximum_head_rad)

    def _joint_state_cb(self, message):
        measured = joint_position(message, self.joint_name)
        if measured is None:
            rospy.logwarn_throttle(
                5.0,
                '[HeadRotationBridge] %s missing/invalid on joint-state topic',
                self.joint_name)
            return
        self._last_joint_position = measured
        self._last_feedback_receipt = rospy.Time.now()
        logical_angle = joint_to_head(
            measured, self.command_sign, self.joint_zero_rad)
        self._feedback_pub.publish(Float64(data=logical_angle))

    def _feedback_is_fresh(self):
        if self._last_feedback_receipt is None:
            return False, float('inf')
        age = max(
            0.0,
            (rospy.Time.now() - self._last_feedback_receipt).to_sec())
        return age <= self.feedback_timeout, age

    def _command_cb(self, message):
        requested = float(message.data)
        if not math.isfinite(requested):
            rospy.logerr('[HeadRotationBridge] rejected non-finite command')
            return

        fresh, age = self._feedback_is_fresh()
        if self.require_fresh_feedback and not fresh:
            detail = 'unavailable' if not math.isfinite(age) else '%.3fs old' % age
            rospy.logerr(
                '[HeadRotationBridge] rejected %.3frad command: %s feedback %s',
                requested, self.joint_name, detail)
            return

        target = requested
        if target < self.minimum_head_rad or target > self.maximum_head_rad:
            if not self.clamp_commands:
                rospy.logerr(
                    '[HeadRotationBridge] rejected %.3frad command outside '
                    '[%.3f, %.3f]', requested, self.minimum_head_rad,
                    self.maximum_head_rad)
                return
            target = min(self.maximum_head_rad,
                         max(self.minimum_head_rad, target))
            rospy.logwarn(
                '[HeadRotationBridge] clamped %.3frad command to %.3frad',
                requested, target)

        joint_target = head_to_joint(
            target, self.command_sign, self.joint_zero_rad)
        trajectory = JointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        point.positions = [joint_target]
        point.time_from_start = rospy.Duration(self.trajectory_duration)
        trajectory.points = [point]
        self._trajectory_pub.publish(trajectory)
        rospy.loginfo(
            '[HeadRotationBridge] command head=%.3frad -> %s=%.3frad '
            '(duration=%.2fs)', target, self.joint_name, joint_target,
            self.trajectory_duration)


def main():
    rospy.init_node('head_rotation_dynamixel_bridge')
    try:
        HeadRotationDynamixelBridge()
    except Exception as exc:
        rospy.logfatal('[HeadRotationBridge] startup failed: %s', exc)
        raise
    rospy.spin()


if __name__ == '__main__':
    main()
