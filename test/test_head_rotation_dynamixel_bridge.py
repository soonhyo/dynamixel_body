#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path

from sensor_msgs.msg import JointState


SCRIPT = (Path(__file__).parents[1] / 'scripts'
          / 'head_rotation_dynamixel_bridge.py')
SPEC = importlib.util.spec_from_file_location('head_rotation_bridge', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_angle_mapping_round_trip_with_sign_and_zero():
    for logical in (-1.2, 0.0, 0.75):
        joint = MODULE.head_to_joint(logical, -1.0, 0.25)
        restored = MODULE.joint_to_head(joint, -1.0, 0.25)
        assert math.isclose(restored, logical, abs_tol=1e-12)


def test_joint_position_requires_named_finite_position():
    message = JointState()
    message.name = ['other', 'body_joint']
    message.position = [0.1, -0.4]
    assert MODULE.joint_position(message, 'body_joint') == -0.4
    assert MODULE.joint_position(message, 'missing') is None

    message.position[1] = float('nan')
    assert MODULE.joint_position(message, 'body_joint') is None


def test_launch_defaults_match_full_pipeline_topics():
    launch = (Path(__file__).parents[1] / 'launch'
              / 'dynamixel_body.launch').read_text()
    assert '/hair_task_manager/head_rotation/command' in launch
    assert '/hair_task_manager/head_rotation/feedback' in launch
    assert 'type="head_rotation_dynamixel_bridge.py"' in launch
    assert 'launch_head_rotation_bridge' in launch
    assert 'default="/dev/dynamixel_body"' in launch


def test_udev_installer_and_template_are_packaged():
    package = Path(__file__).parents[1]
    installer = package / 'scripts' / 'apply_dynamixel_body_udev.sh'
    template = package / 'udev' / '99-dynamixel-body.rules.template'
    assert installer.is_file()
    assert template.is_file()
    text = template.read_text()
    assert 'ATTRS{serial}=="@SERIAL@"' in text
    assert 'SYMLINK+="@SYMLINK@"' in text
