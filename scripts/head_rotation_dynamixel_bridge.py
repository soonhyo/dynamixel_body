#!/usr/bin/env python3
"""Bridge the hair pipeline head-angle API to dynamixel_general_hw.

The task pipeline deliberately exposes a hardware-independent pair of
``std_msgs/Float64`` topics.  ``dynamixel_general_hw`` instead accepts a
``trajectory_msgs/JointTrajectory`` and reports ``sensor_msgs/JointState``.
This node is the single owner of that conversion, including the installation
specific zero and direction convention.
"""

import math
import threading

import rospy
from dynamic_reconfigure.server import Server
from dynamixel_body.cfg import HeadRotationBridgeConfig
from dynamixel_body.srv import (SetCommandSign, SetCommandSignResponse,
                                SetJointZero, SetJointZeroResponse)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def is_finite(value):
    """Python 2/3 compatible finite-number check."""
    return not math.isnan(float(value)) and not math.isinf(float(value))


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
    return value if is_finite(value) else None


def named_value(message, joint_name, field):
    """Return a finite named JointState field value, or ``None``."""
    try:
        index = list(message.name).index(str(joint_name))
    except ValueError:
        return None
    values = getattr(message, field)
    if index >= len(values):
        return None
    value = float(values[index])
    return value if is_finite(value) else None


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
        self.move_to_zero_on_start = bool(rospy.get_param(
            '~startup/move_to_zero', True))
        self.startup_timeout = float(rospy.get_param(
            '~startup/timeout_s', 30.0))
        self.calibration_command_quiet = float(rospy.get_param(
            '~calibration_command_quiet_s', 2.0))
        self.calibration_max_velocity = float(rospy.get_param(
            '~calibration_max_velocity_rad_s', 0.01))
        self._config_lock = threading.RLock()

        if not self.joint_name:
            raise ValueError('joint_name must not be empty')
        if not is_finite(self.command_sign) or abs(self.command_sign) < 1e-12:
            raise ValueError('command_sign must be finite and nonzero')
        if not is_finite(self.joint_zero_rad):
            raise ValueError('joint_zero_rad must be finite')
        if not (is_finite(self.minimum_head_rad)
                and is_finite(self.maximum_head_rad)
                and self.minimum_head_rad < self.maximum_head_rad):
            raise ValueError('head limits must be finite and min < max')
        if not is_finite(self.trajectory_duration) or self.trajectory_duration <= 0.0:
            raise ValueError('trajectory_duration_s must be positive')
        if not is_finite(self.feedback_timeout) or self.feedback_timeout <= 0.0:
            raise ValueError('feedback_timeout_s must be positive')
        if not is_finite(self.startup_timeout) or self.startup_timeout <= 0.0:
            raise ValueError('startup/timeout_s must be positive')
        if (self.move_to_zero_on_start
                and not self.minimum_head_rad <= 0.0 <= self.maximum_head_rad):
            raise ValueError('logical zero must be within the head limits')
        if (not is_finite(self.calibration_command_quiet)
                or self.calibration_command_quiet < 0.0):
            raise ValueError('calibration_command_quiet_s must be nonnegative')
        if (not is_finite(self.calibration_max_velocity)
                or self.calibration_max_velocity < 0.0):
            raise ValueError(
                'calibration_max_velocity_rad_s must be nonnegative')

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
        self._last_joint_velocity = None
        self._last_command_receipt = None
        self._dynamic_initialized = False
        self._startup_lock = threading.RLock()
        self._startup_pending = self.move_to_zero_on_start
        self._startup_timer = None
        self._trajectory_pub = rospy.Publisher(
            trajectory_topic, JointTrajectory, queue_size=1)
        self._feedback_pub = rospy.Publisher(
            feedback_topic, Float64, queue_size=1, latch=True)
        self._joint_sub = rospy.Subscriber(
            joint_state_topic, JointState, self._joint_state_cb, queue_size=1)
        self._command_sub = rospy.Subscriber(
            command_topic, Float64, self._command_cb, queue_size=1)
        self._zero_here_service = rospy.Service(
            '~zero_here', Trigger, self._zero_here_cb)
        self._set_zero_service = rospy.Service(
            '~set_zero', SetJointZero, self._set_zero_cb)
        self._set_direction_service = rospy.Service(
            '~set_direction', SetCommandSign, self._set_direction_cb)
        self._dynamic_server = Server(
            HeadRotationBridgeConfig, self._dynamic_config_cb)

        if self._startup_pending:
            self._startup_deadline = (
                rospy.Time.now() + rospy.Duration(self.startup_timeout))
            self._startup_timer = rospy.Timer(
                rospy.Duration(0.1), self._startup_zero_cb)

        rospy.loginfo(
            '[HeadRotationBridge] ready joint=%s command=%s feedback=%s '
            'trajectory=%s states=%s sign=%+.3f zero=%.4f limits=[%.3f, %.3f] '
            'startup_zero=%s',
            self.joint_name, command_topic, feedback_topic, trajectory_topic,
            joint_state_topic, self.command_sign, self.joint_zero_rad,
            self.minimum_head_rad, self.maximum_head_rad,
            self.move_to_zero_on_start)

    def _dynamic_config_cb(self, config, _level):
        requested_zero = float(config.joint_zero_rad)
        requested_sign = float(config.command_sign)
        with self._config_lock:
            if not is_finite(requested_zero):
                rospy.logerr(
                    '[HeadRotationBridge] rejected non-finite joint_zero_rad')
                config.joint_zero_rad = self.joint_zero_rad
                return config
            if not is_finite(requested_sign) or abs(requested_sign) < 1e-12:
                rospy.logerr(
                    '[HeadRotationBridge] rejected command_sign=%s; use +1 or -1',
                    config.command_sign)
                config.command_sign = int(math.copysign(1, self.command_sign))
                return config

            changed = (requested_zero != self.joint_zero_rad
                       or requested_sign != self.command_sign)
            if changed and self._dynamic_initialized:
                safe, reason = self._calibration_is_safe()
                if not safe:
                    rospy.logerr(
                        '[HeadRotationBridge] rejected calibration update: %s',
                        reason)
                    config.joint_zero_rad = self.joint_zero_rad
                    config.command_sign = int(self.command_sign)
                    return config
            self.joint_zero_rad = requested_zero
            self.command_sign = requested_sign
            measured = self._last_joint_position
            self._dynamic_initialized = True

        if changed:
            rospy.logwarn(
                '[HeadRotationBridge] calibration updated dynamically: '
                'sign=%+.0f zero=%.6f rad', requested_sign, requested_zero)
            if measured is not None:
                self._feedback_pub.publish(Float64(data=joint_to_head(
                    measured, requested_sign, requested_zero)))
        return config

    def _calibration_is_safe(self):
        fresh, age = self._feedback_is_fresh()
        if not fresh:
            detail = 'unavailable' if not is_finite(age) else '%.3fs old' % age
            return False, '%s feedback %s' % (self.joint_name, detail)
        if self._last_joint_velocity is None:
            return False, '%s velocity unavailable' % self.joint_name
        if abs(self._last_joint_velocity) > self.calibration_max_velocity:
            return False, '%s is moving at %.4frad/s' % (
                self.joint_name, self._last_joint_velocity)
        if self._last_command_receipt is not None:
            command_age = max(
                0.0, (rospy.Time.now() - self._last_command_receipt).to_sec())
            required_quiet = max(
                self.calibration_command_quiet, self.trajectory_duration)
            if command_age < required_quiet:
                return False, 'command received %.3fs ago' % command_age
        return True, 'safe'

    def _update_calibration(self, zero=None, sign=None):
        update = {}
        if zero is not None:
            update['joint_zero_rad'] = float(zero)
        if sign is not None:
            update['command_sign'] = int(sign)
        result = self._dynamic_server.update_configuration(update)
        return ((zero is None or result.joint_zero_rad == float(zero))
                and (sign is None or result.command_sign == int(sign)))

    def _zero_here_cb(self, _request):
        with self._config_lock:
            measured = self._last_joint_position
        if measured is None:
            return TriggerResponse(False, 'body_joint feedback unavailable')
        if not self._update_calibration(zero=measured):
            return TriggerResponse(False, 'unsafe calibration change; see node log')
        return TriggerResponse(
            True, 'logical zero set to %.6frad' % measured)

    def _set_zero_cb(self, request):
        requested = float(request.joint_zero_rad)
        if not is_finite(requested):
            return SetJointZeroResponse(False, 'joint_zero_rad must be finite')
        if not self._update_calibration(zero=requested):
            return SetJointZeroResponse(
                False, 'unsafe calibration change; see node log')
        return SetJointZeroResponse(
            True, 'joint_zero_rad set to %.6frad' % requested)

    def _set_direction_cb(self, request):
        requested = int(request.command_sign)
        if requested not in (-1, 1):
            return SetCommandSignResponse(
                False, 'command_sign must be +1 or -1')
        if not self._update_calibration(sign=requested):
            return SetCommandSignResponse(
                False, 'unsafe calibration change; see node log')
        return SetCommandSignResponse(
            True, 'command_sign set to %+d' % requested)

    def _joint_state_cb(self, message):
        measured = joint_position(message, self.joint_name)
        if measured is None:
            rospy.logwarn_throttle(
                5.0,
                '[HeadRotationBridge] %s missing/invalid on joint-state topic',
                self.joint_name)
            return
        self._last_joint_position = measured
        self._last_joint_velocity = named_value(
            message, self.joint_name, 'velocity')
        self._last_feedback_receipt = rospy.Time.now()
        with self._config_lock:
            command_sign = self.command_sign
            joint_zero_rad = self.joint_zero_rad
        logical_angle = joint_to_head(
            measured, command_sign, joint_zero_rad)
        self._feedback_pub.publish(Float64(data=logical_angle))

    def _feedback_is_fresh(self):
        if self._last_feedback_receipt is None:
            return False, float('inf')
        age = max(
            0.0,
            (rospy.Time.now() - self._last_feedback_receipt).to_sec())
        return age <= self.feedback_timeout, age

    def _finish_startup(self):
        self._startup_pending = False
        timer = self._startup_timer
        self._startup_timer = None
        if timer is not None:
            timer.shutdown()

    def _startup_zero_cb(self, _event):
        with self._startup_lock:
            if not self._startup_pending:
                return
            if rospy.Time.now() >= self._startup_deadline:
                self._finish_startup()
                rospy.logerr(
                    '[HeadRotationBridge] startup zero command timed out after '
                    '%.1fs while waiting for controller connection and fresh '
                    '%s feedback', self.startup_timeout, self.joint_name)
                return
            if self._trajectory_pub.get_num_connections() < 1:
                return
            fresh, _age = self._feedback_is_fresh()
            if not fresh:
                return
            if self._publish_head_target(0.0):
                self._finish_startup()
                rospy.loginfo(
                    '[HeadRotationBridge] startup move to logical 0deg sent')

    def _command_cb(self, message):
        with self._startup_lock:
            if self._publish_head_target(message.data):
                if self._startup_pending:
                    self._finish_startup()
                    rospy.loginfo(
                        '[HeadRotationBridge] startup zero skipped because an '
                        'external command was accepted first')

    def _publish_head_target(self, requested):
        self._last_command_receipt = rospy.Time.now()
        requested = float(requested)
        if not is_finite(requested):
            rospy.logerr('[HeadRotationBridge] rejected non-finite command')
            return False

        fresh, age = self._feedback_is_fresh()
        if self.require_fresh_feedback and not fresh:
            detail = 'unavailable' if not is_finite(age) else '%.3fs old' % age
            rospy.logerr(
                '[HeadRotationBridge] rejected %.3frad command: %s feedback %s',
                requested, self.joint_name, detail)
            return False

        target = requested
        if target < self.minimum_head_rad or target > self.maximum_head_rad:
            if not self.clamp_commands:
                rospy.logerr(
                    '[HeadRotationBridge] rejected %.3frad command outside '
                    '[%.3f, %.3f]', requested, self.minimum_head_rad,
                    self.maximum_head_rad)
                return False
            target = min(self.maximum_head_rad,
                         max(self.minimum_head_rad, target))
            rospy.logwarn(
                '[HeadRotationBridge] clamped %.3frad command to %.3frad',
                requested, target)

        with self._config_lock:
            command_sign = self.command_sign
            joint_zero_rad = self.joint_zero_rad
        joint_target = head_to_joint(
            target, command_sign, joint_zero_rad)
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
        return True


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
