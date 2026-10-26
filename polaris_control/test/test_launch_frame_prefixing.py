"""Regression tests for namespaced TF frame launch arguments."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters
import pytest
import yaml


LAUNCH_DIRECTORY = Path(__file__).parents[1] / 'launch'
POLARIS_ROOT = Path(__file__).parents[2]


class _PackageShare:
    def perform(self, _context):
        return str(POLARIS_ROOT / 'polaris_control')


def _load_launch_module(filename):
    module_name = filename.removesuffix('.launch.py').replace('.', '_')
    spec = importlib.util.spec_from_file_location(
        module_name, LAUNCH_DIRECTORY / filename
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(
    params=['navigation.launch.py', 'follower_control.launch.py']
)
def launch_module(request, monkeypatch):
    module = _load_launch_module(request.param)
    monkeypatch.setattr(module, 'FindPackageShare', lambda _package: _PackageShare())
    return module


@pytest.mark.parametrize(
    ('robot_name', 'frame', 'prefix_enabled', 'expected'),
    [
        ('scout_1', 'body', True, 'scout_1/body'),
        ('scout_1', 'scout_1/body', True, 'scout_1/body'),
        ('scout_1', 'map', True, 'scout_1/map'),
        ('scout_1', 'scout_1/map', True, 'scout_1/map'),
        ('scout_1', 'scout_10/body', True, 'scout_1/scout_10/body'),
        ('scout_1', 'world', True, 'world'),
        ('scout_1', 'sim_world', True, 'sim_world'),
        ('pioneer_1', 'sim_world', True, 'sim_world'),
        ('scout_1', 'body', False, 'body'),
        ('scout_1', '', True, ''),
    ],
)
def test_resolve_tf_frame_is_idempotent(
    launch_module, robot_name, frame, prefix_enabled, expected
):
    assert launch_module._resolve_tf_frame(
        robot_name, frame, prefix_enabled
    ) == expected


@pytest.mark.parametrize('robot_name', ['', 'fleet/scout_1', 'scout-1'])
def test_robot_name_must_be_one_ros_token(launch_module, robot_name):
    with pytest.raises(RuntimeError):
        launch_module._validate_robot_name(robot_name)


@pytest.mark.parametrize(
    ('namespace_tf', 'expected'),
    [
        (False, []),
        (True, [('/tf', 'tf'), ('/tf_static', 'tf_static')]),
    ],
)
def test_namespace_tf_independently_controls_tf_transport(
    launch_module, namespace_tf, expected
):
    assert launch_module._tf_remappings(namespace_tf) == expected
    assert launch_module._tf_topic(namespace_tf) == (
        'tf' if namespace_tf else '/tf'
    )


@pytest.mark.parametrize(
    ('namespace_tf', 'prefix_tf_frames', 'expected_remaps', 'expected_frame'),
    [
        (False, False, [], 'body'),
        (True, False, [('/tf', 'tf'), ('/tf_static', 'tf_static')], 'body'),
        (False, True, [], 'scout_1/body'),
        (
            True,
            True,
            [('/tf', 'tf'), ('/tf_static', 'tf_static')],
            'scout_1/body',
        ),
    ],
)
def test_legacy_flag_combinations_are_independent(
    launch_module,
    namespace_tf,
    prefix_tf_frames,
    expected_remaps,
    expected_frame,
):
    assert launch_module._tf_remappings(namespace_tf) == expected_remaps
    assert (
        launch_module._resolve_tf_frame(
            'scout_1', 'body', prefix_tf_frames
        )
        == expected_frame
    )


def test_launch_keeps_legacy_flags_and_requires_robot_name(launch_module):
    description = launch_module.generate_launch_description()
    declarations = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }

    assert 'robot_namespace' not in declarations
    assert declarations['robot_name'].default_value is None
    context = LaunchContext()
    assert perform_substitutions(
        context, declarations['namespace_tf'].default_value
    ) == 'false'
    assert perform_substitutions(
        context, declarations['prefix_tf_frames'].default_value
    ) == 'true'


def test_parameter_files_keep_robot_interfaces_relative_and_frames_local():
    config_files = [
        *sorted((POLARIS_ROOT / 'polaris_control' / 'config').glob('*.yaml')),
        *sorted((POLARIS_ROOT / 'polaris_planning' / 'config').glob('*.yaml')),
    ]

    def check(value, path):
        if isinstance(value, dict):
            for key, child in value.items():
                child_path = (*path, key)
                if (
                    isinstance(child, str)
                    and ('topic' in key or 'service' in key)
                ):
                    assert not child.startswith('/'), (
                        f'{child_path} in {config_file} must be relative'
                    )
                check(child, child_path)
        elif isinstance(value, list):
            for index, child in enumerate(value):
                check(child, (*path, index))

    for config_file in config_files:
        data = yaml.safe_load(config_file.read_text(encoding='utf-8'))
        check(data, ())
        text = config_file.read_text(encoding='utf-8')
        assert 'sim_world' not in text
        assert '{robot_name}' not in text
        assert 'prefix_tf_frames:' not in text


def test_rviz_topic_values_are_relative():
    rviz_files = sorted(POLARIS_ROOT.glob('*/config/*.rviz'))

    assert rviz_files
    for rviz_file in rviz_files:
        for line in rviz_file.read_text(encoding='utf-8').splitlines():
            stripped = line.strip()
            if stripped.startswith('Value: /'):
                pytest.fail(f'{rviz_file} has an absolute RViz topic: {stripped}')


@pytest.mark.parametrize('namespace_tf', [False, True])
def test_launch_setup_namespaces_nodes_without_owning_hardware_tf(
    launch_module, namespace_tf
):
    context = LaunchContext()
    context.launch_configurations.update({
        'robot_name': 'scout_1',
        'params_file': 'pioneer_params.yaml',
        'namespace_tf': str(namespace_tf).lower(),
        'prefix_tf_frames': 'true',
        'use_sim_time': 'false',
        'tf_robot_pose': '',
        'tf_reference_frame': '',
        'tf_inertial_link': '',
        'speed_ref': '',
        'convergence_gain': '',
        'add_measurement_noise': 'false',
        'noise_mean_position': '0.0',
        'noise_stddev_position': '0.0',
        'noise_mean_yaw': '0.0',
        'noise_stddev_yaw': '0.0',
        'kp_pos': '1.0',
        'kp_orient': '1.0',
    })

    actions = launch_module._launch_setup(context)
    assert actions
    assert all(action.node_package != 'tf2_ros' for action in actions)
    for action in actions:
        action._perform_substitutions(context)
        assert action.expanded_node_namespace == '/scout_1'
        tf_remaps = [
            remap
            for remap in (action.expanded_remapping_rules or [])
            if remap[0] in ('/tf', '/tf_static')
        ]
        assert bool(tf_remaps) is namespace_tf

        evaluated = evaluate_parameters(context, action._Node__parameters)
        parameters = {}
        for parameter_set in evaluated:
            if isinstance(parameter_set, dict):
                parameters.update(parameter_set)
        if 'tf_robot_pose' in parameters:
            assert parameters['tf_robot_pose'] == 'scout_1/base_link'
        if 'tf_reference_frame' in parameters:
            assert parameters['tf_reference_frame'] == 'scout_1/map'
        if 'tf_inertial_link' in parameters:
            assert parameters['tf_inertial_link'] == 'scout_1/map'
        pose_topic_key = (
            'pose_topic_name'
            if 'pose_topic_name' in parameters
            else 'pose_topic'
        )
        if parameters.get('pose_topic_type') == 'TFMessage':
            assert parameters[pose_topic_key] == (
                'tf' if namespace_tf else '/tf'
            )


@pytest.mark.parametrize(
    ('robot_name', 'params_file'),
    [
        ('scout_1', 'scout_params.yaml'),
        ('pioneer_1', 'pioneer_params.yaml'),
    ],
)
def test_launch_setup_keeps_sim_world_global(
    launch_module, robot_name, params_file
):
    context = LaunchContext()
    context.launch_configurations.update({
        'robot_name': robot_name,
        'params_file': params_file,
        'namespace_tf': 'true',
        'prefix_tf_frames': 'true',
        'use_sim_time': 'true',
        'tf_robot_pose': robot_name,
        'tf_reference_frame': 'sim_world',
        'tf_inertial_link': 'sim_world',
        'speed_ref': '',
        'convergence_gain': '',
        'add_measurement_noise': 'false',
        'noise_mean_position': '0.0',
        'noise_stddev_position': '0.0',
        'noise_mean_yaw': '0.0',
        'noise_stddev_yaw': '0.0',
        'kp_pos': '1.0',
        'kp_orient': '1.0',
    })

    found_global_parent = False
    for action in launch_module._launch_setup(context):
        action._perform_substitutions(context)
        assert action.expanded_node_namespace == f'/{robot_name}'

        evaluated = evaluate_parameters(context, action._Node__parameters)
        parameters = {}
        for parameter_set in evaluated:
            if isinstance(parameter_set, dict):
                parameters.update(parameter_set)

        assert parameters['tf_robot_pose'] == robot_name
        if 'tf_reference_frame' in parameters:
            assert parameters['tf_reference_frame'] == 'sim_world'
            found_global_parent = True
        if 'tf_inertial_link' in parameters:
            assert parameters['tf_inertial_link'] == 'sim_world'
            found_global_parent = True

    assert found_global_parent
