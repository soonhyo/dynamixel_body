#!/usr/bin/env python3

import imp
import os

from sensor_msgs.msg import JointState


PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT = os.path.join(PACKAGE, 'scripts',
                      'head_rotation_dynamixel_bridge.py')
MODULE = imp.load_source('head_rotation_bridge', SCRIPT)


def test_angle_mapping_round_trip_with_sign_and_zero():
    for logical in (-1.2, 0.0, 0.75):
        joint = MODULE.head_to_joint(logical, -1.0, 0.25)
        restored = MODULE.joint_to_head(joint, -1.0, 0.25)
        assert abs(restored - logical) <= 1e-12


def test_joint_position_requires_named_finite_position():
    message = JointState()
    message.name = ['other', 'body_joint']
    message.position = [0.1, -0.4]
    assert MODULE.joint_position(message, 'body_joint') == -0.4
    assert MODULE.joint_position(message, 'missing') is None

    message.position[1] = float('nan')
    assert MODULE.joint_position(message, 'body_joint') is None


def test_named_value_requires_available_finite_field():
    message = JointState()
    message.name = ['body_joint']
    message.velocity = [0.02]
    assert MODULE.named_value(message, 'body_joint', 'velocity') == 0.02
    message.velocity = []
    assert MODULE.named_value(message, 'body_joint', 'velocity') is None


def test_launch_defaults_match_full_pipeline_topics():
    launch_path = os.path.join(PACKAGE, 'launch', 'dynamixel_body.launch')
    with open(launch_path) as stream:
        launch = stream.read()
    assert '/hair_task_manager/head_rotation/command' in launch
    assert '/hair_task_manager/head_rotation/feedback' in launch
    assert 'type="head_rotation_dynamixel_bridge.py"' in launch
    assert 'launch_head_rotation_bridge' in launch
    assert 'default="/dev/dynamixel_body"' in launch


def test_dynamic_reconfigure_exposes_calibration_parameters():
    config_path = os.path.join(PACKAGE, 'cfg', 'HeadRotationBridge.cfg')
    cmake_path = os.path.join(PACKAGE, 'CMakeLists.txt')
    with open(config_path) as stream:
        config = stream.read()
    with open(cmake_path) as stream:
        cmake = stream.read()
    assert '"joint_zero_rad"' in config
    assert '"command_sign"' in config
    assert 'cfg/HeadRotationBridge.cfg' in cmake
    assert os.path.isfile(os.path.join(PACKAGE, 'srv', 'SetJointZero.srv'))
    assert os.path.isfile(os.path.join(PACKAGE, 'srv', 'SetCommandSign.srv'))


def test_udev_installer_and_template_are_packaged():
    installer = os.path.join(PACKAGE, 'scripts',
                             'apply_dynamixel_body_udev.sh')
    template = os.path.join(PACKAGE, 'udev',
                            '99-dynamixel-body.rules.template')
    assert os.path.isfile(installer)
    assert os.path.isfile(template)
    with open(template) as stream:
        text = stream.read()
    assert 'ATTRS{serial}=="@SERIAL@"' in text
    assert 'SYMLINK+="@SYMLINK@"' in text
