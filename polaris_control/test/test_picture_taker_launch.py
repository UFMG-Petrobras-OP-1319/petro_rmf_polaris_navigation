"""picture_taker is started next to the follower with the right cameras per runtime."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch_ros.utilities import evaluate_parameters
import pytest


LAUNCH_FILE = Path(__file__).parents[1] / 'launch' / 'follower_control.launch.py'
POLARIS_ROOT = Path(__file__).parents[2]


class _PackageShare:
    def perform(self, _context):
        return str(POLARIS_ROOT / 'polaris_control')


@pytest.fixture
def launch_module(monkeypatch):
    spec = importlib.util.spec_from_file_location('follower_control_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'FindPackageShare', lambda _package: _PackageShare())
    return module


def _runtime_parameters(module, params_file, use_sim_time):
    context = LaunchContext()
    context.launch_configurations.update({
        'robot_name': 'scout_1',
        'params_file': params_file,
        'namespace_tf': 'true',
        'prefix_tf_frames': 'true',
        'use_sim_time': str(use_sim_time).lower(),
        'tf_robot_pose': 'scout_1' if use_sim_time else '',
        'tf_inertial_link': 'sim_world' if use_sim_time else '',
        'kp_pos': '1.0',
        'kp_orient': '1.0',
    })
    actions = module._launch_setup(context)
    pictures = [action for action in actions if action.node_executable == 'picture_taker.py']
    assert len(pictures) == 1
    action = pictures[0]
    action._perform_substitutions(context)
    assert action.expanded_node_namespace == '/scout_1'
    parameters = {}
    for parameter_set in evaluate_parameters(context, action._Node__parameters):
        if isinstance(parameter_set, dict):
            parameters.update(parameter_set)
    return parameters


@pytest.mark.parametrize('params_file', ['scout_params.yaml', 'pioneer_params.yaml'])
def test_simulation_uses_the_snapshot_services(launch_module, params_file):
    parameters = _runtime_parameters(launch_module, params_file, use_sim_time=True)

    assert list(parameters['cameras']) == ['default', 'inspection']
    assert parameters['default.service'] == 'fixed_camera_snapshot/take_photo'
    assert parameters['inspection.service'] == 'ptz_camera_snapshot/take_photo'
    assert parameters['inspection.joint_state_topic'] == 'joint_states'
    assert not parameters.get('inspection.return_to_forward', False)
    assert not parameters.get('inspection.aim_pan_cmd_topic', '')
    assert parameters['tf_inertial_link'] == 'sim_world'
    assert not any(key.startswith('hardware') for key in parameters)


@pytest.mark.parametrize('params_file', ['scout_params.yaml', 'pioneer_params.yaml'])
def test_hardware_uses_the_hikvision_driver(launch_module, params_file):
    parameters = _runtime_parameters(launch_module, params_file, use_sim_time=False)

    assert parameters['inspection.service'] == 'hk_camera_rtsp_pub/take_photo'
    assert parameters['inspection.joint_state_topic'] == 'hk_camera_ptz/joint_states'
    if params_file == 'scout_params.yaml':
        assert parameters['inspection.return_to_forward']
        assert parameters['inspection.forward_pan_rad'] == 0.0
        assert parameters['inspection.forward_tilt_rad'] == 0.0
        assert parameters['inspection.forward_zoom'] == 1.0
        assert parameters['inspection.aim_pan_cmd_topic'] == 'pan_camera_cmd'
        assert parameters['inspection.aim_tilt_cmd_topic'] == 'tilt_camera_cmd'
        assert parameters['ready_timeout_sec'] == 8.0
        assert parameters['still_window_sec'] == 2.0
        assert parameters['inspection.settle_sec'] == 3.0
    else:
        assert not parameters.get('inspection.return_to_forward', False)
        assert not parameters.get('inspection.aim_pan_cmd_topic', '')
    # Untouched by the overrides
    assert parameters['inspection.tolerance_rad'] == 0.05
    assert parameters['default.service'] == 'fixed_camera_snapshot/take_photo'
    assert parameters['tf_robot_pose'] == 'scout_1/base_link'


def test_params_without_cameras_start_no_picture_taker(launch_module):
    context = LaunchContext()
    context.launch_configurations.update({
        'robot_name': 'espeleo_1',
        'params_file': 'espeleo_params.yaml',
        'namespace_tf': 'true',
        'prefix_tf_frames': 'true',
        'use_sim_time': 'true',
        'tf_robot_pose': '',
        'tf_inertial_link': '',
        'kp_pos': '1.0',
        'kp_orient': '1.0',
    })
    actions = launch_module._launch_setup(context)
    assert [action.node_executable for action in actions] == ['follower_control.py']


def test_merge_is_deep(launch_module):
    merged = launch_module._runtime_parameters(
        {'a': 1, 'cam': {'x': 1, 'y': 2}, 'hardware': {'cam': {'y': 3}}}, use_sim_time=False)
    assert merged == {'a': 1, 'cam': {'x': 1, 'y': 3}}
