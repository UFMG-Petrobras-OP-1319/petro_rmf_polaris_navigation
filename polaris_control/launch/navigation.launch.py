"""Launch the Polaris planner and vector-field controller."""

import logging
import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

_logger = logging.getLogger('navigation.launch')


_ROS_NAME_TOKEN = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')


def _validate_robot_name(value):
    if not value or not _ROS_NAME_TOKEN.fullmatch(value):
        raise RuntimeError('robot_name must be one valid ROS name token')
    return value


def _as_bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _tf_remappings(namespace_tf):
    return [('/tf', 'tf'), ('/tf_static', 'tf_static')] if namespace_tf else []


def _tf_topic(namespace_tf):
    return 'tf' if namespace_tf else '/tf'


def _resolve_tf_frame(robot_name, frame, prefix_tf_frames):
    """Apply ``robot_name`` to a TF frame at most once."""
    if not frame:
        return frame
    if not prefix_tf_frames:
        return frame
    if frame == robot_name or frame.startswith(f'{robot_name}/'):
        return frame
    return f'{robot_name}/{frame}'


def _launch_setup(context, *_args, **_kwargs):
    package_name = 'polaris_control'

    params_file = LaunchConfiguration('params_file').perform(context)
    robot_name = _validate_robot_name(
        LaunchConfiguration('robot_name').perform(context)
    )
    namespace_tf = _as_bool(
        LaunchConfiguration('namespace_tf').perform(context)
    )
    use_sim_time = _as_bool(
        LaunchConfiguration('use_sim_time').perform(context)
    )
    tf_robot_pose_arg = LaunchConfiguration('tf_robot_pose').perform(context)
    speed_ref = LaunchConfiguration('speed_ref').perform(context)
    convergence_gain = LaunchConfiguration('convergence_gain').perform(context)
    add_measurement_noise = (
        LaunchConfiguration('add_measurement_noise')
        .perform(context)
        .lower() == 'true'
    )
    prefix_tf_frames = _as_bool(
        LaunchConfiguration('prefix_tf_frames').perform(context)
    )
    noise_mean_position = float(
        LaunchConfiguration('noise_mean_position').perform(context)
    )
    noise_stddev_position = float(
        LaunchConfiguration('noise_stddev_position').perform(context)
    )
    noise_mean_yaw = float(
        LaunchConfiguration('noise_mean_yaw').perform(context)
    )
    noise_stddev_yaw = float(
        LaunchConfiguration('noise_stddev_yaw').perform(context)
    )
    tf_reference_frame_arg = LaunchConfiguration('tf_reference_frame').perform(
        context
    )

    pkg_share = FindPackageShare(package_name).perform(context)
    param_config_file = os.path.join(pkg_share, 'config', params_file)

    with open(param_config_file, encoding='utf-8') as parameter_file:
        parameter_config = yaml.safe_load(parameter_file)

    controller_parameters = parameter_config['controller']['ros__parameters']
    planner_parameters = parameter_config['planner']['ros__parameters']

    controller_robot_frame = _resolve_tf_frame(
        robot_name,
        tf_robot_pose_arg or controller_parameters.get('tf_robot_pose', ''),
        prefix_tf_frames,
    )
    planner_robot_frame = _resolve_tf_frame(
        robot_name,
        tf_robot_pose_arg or planner_parameters.get('tf_robot_pose', ''),
        prefix_tf_frames,
    )
    controller_reference_frame = _resolve_tf_frame(
        robot_name,
        tf_reference_frame_arg
        or controller_parameters.get('tf_reference_frame', ''),
        prefix_tf_frames,
    )
    planner_reference_frame = _resolve_tf_frame(
        robot_name,
        tf_reference_frame_arg
        or planner_parameters.get('tf_reference_frame', ''),
        prefix_tf_frames,
    )

    # YAML file sets all defaults; build optional override dictionaries for frame
    # names so docker-compose / CI can tune them without touching the YAML.
    controller_overrides = {
        'add_measurement_noise': add_measurement_noise,
        'noise_mean_position': noise_mean_position,
        'noise_stddev_position': noise_stddev_position,
        'noise_mean_yaw': noise_mean_yaw,
        'noise_stddev_yaw': noise_stddev_yaw,
        'tf_robot_pose': controller_robot_frame,
        'tf_reference_frame': controller_reference_frame,
        'path_topic_name': 'ref_path',
        'cmd_vel_topic_name': 'vec_to_follow',
        'closest_obstacle_topic_name': 'closest_obstacle',
        'is_path_closed_service_name': 'is_path_closed',
    }
    planner_overrides = {
        'tf_robot_pose': planner_robot_frame,
        'tf_reference_frame': planner_reference_frame,
        'path_topic_name': 'ref_path',
        'visualization_topic_name': 'visual_path',
        'clicked_point_topic_name': 'goal_pose',
        'start_service_name': 'start_planner',
        'clear_service_name': 'clear_planner',
        'is_path_closed_service_name': 'is_path_closed',
        'remove_last_point_service_name': 'remove_last_point',
    }
    if controller_parameters.get('pose_topic_type') == 'TFMessage':
        controller_overrides['pose_topic_name'] = _tf_topic(namespace_tf)
    if planner_parameters.get('pose_topic_type') == 'TFMessage':
        planner_overrides['pose_topic_name'] = _tf_topic(namespace_tf)

    if speed_ref:
        controller_overrides['speed_ref'] = float(speed_ref)
    else:
        _logger.warning(
            '⚠️  speed_ref not set via launch arg — '
            'falling back to value in %s. Pass speed_ref:=<value> to override.',
            params_file,
        )

    if convergence_gain:
        controller_overrides['convergence_gain'] = float(convergence_gain)
    else:
        _logger.warning(
            '⚠️  convergence_gain not set via launch arg — '
            'falling back to value in %s. Pass convergence_gain:=<value> to override.',
            params_file,
        )

    tf_remappings = _tf_remappings(namespace_tf)

    controller_node = Node(
        package=package_name,
        executable='vector_field_controller',
        name='controller',
        namespace=robot_name,
        output='screen',
        parameters=[
            controller_parameters,
            {'use_sim_time': use_sim_time},
            controller_overrides,
        ],
        remappings=tf_remappings,
    )

    planner_node = Node(
        package='polaris_planning',
        executable='path_from_points',
        name='planner',
        namespace=robot_name,
        output='screen',
        parameters=[
            planner_parameters,
            {'use_sim_time': use_sim_time},
            planner_overrides,
        ],
        remappings=tf_remappings,
    )

    return [controller_node, planner_node]


def generate_launch_description():
    """Return the configurable Polaris navigation launch description."""
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='pioneer_params.yaml',
        description=(
            'Controller params YAML filename under polaris_control/config/'
        ),
    )

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        description='Robot identity applied as the Polaris ROS namespace.',
    )

    declare_namespace_tf = DeclareLaunchArgument(
        'namespace_tf',
        default_value='false',
        description=(
            'Remap /tf and /tf_static beneath robot_name. Leave false only '
            'for legacy integrations.'
        ),
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use the simulation clock for all launched nodes.',
    )

    # TF child frame identifying the robot body (e.g. pioneer, scout_mini).
    # Empty string means: use whatever is set in the params YAML.
    declare_tf_robot_pose = DeclareLaunchArgument(
        'tf_robot_pose',
        default_value='',
        description=(
            'TF child frame for the robot body (e.g. pioneer, scout_mini). '
            'If empty, the value from params_file YAML is used.'
        ),
    )

    # TF parent (world/reference) frame used by the controller and planner.
    # Empty string means: use whatever is set in the params YAML.
    declare_tf_reference_frame = DeclareLaunchArgument(
        'tf_reference_frame',
        default_value='',
        description=(
            'TF parent (world/reference) frame (e.g. map). '
            'If empty, the value from params_file YAML is used.'
        ),
    )

    # Reference cruise speed for the vector-field controller (m/s).
    # Empty string means: use whatever is set in the params YAML.
    declare_speed_ref = DeclareLaunchArgument(
        'speed_ref',
        default_value='',
        description=(
            'Reference cruise speed for the vector-field controller (m/s). '
            'If empty, the value from params_file YAML is used.'
        ),
    )

    # Convergence gain shaping how fast the field pulls the robot onto the path.
    # Empty string means: use whatever is set in the params YAML.
    declare_convergence_gain = DeclareLaunchArgument(
        'convergence_gain',
        default_value='',
        description=(
            'Convergence gain for the vector-field controller. '
            'If empty, the value from params_file YAML is used.'
        ),
    )

    declare_add_measurement_noise = DeclareLaunchArgument(
        'add_measurement_noise',
        default_value='false',
        description=(
            'Enable Gaussian measurement-noise injection in updateRobotPose '
            '(controller node).'
        ),
    )

    declare_noise_mean_position = DeclareLaunchArgument(
        'noise_mean_position',
        default_value='0.0',
        description='Mean of the Gaussian noise added to x/y position measurements (m).',
    )

    declare_noise_stddev_position = DeclareLaunchArgument(
        'noise_stddev_position',
        default_value='0.0',
        description=(
            'Standard deviation of Gaussian noise added to x/y position '
            'measurements (m).'
        ),
    )

    declare_noise_mean_yaw = DeclareLaunchArgument(
        'noise_mean_yaw',
        default_value='0.0',
        description='Mean of the Gaussian noise added to the yaw measurement (rad).',
    )

    declare_noise_stddev_yaw = DeclareLaunchArgument(
        'noise_stddev_yaw',
        default_value='0.0',
        description='Standard deviation of the Gaussian noise added to the yaw measurement (rad).',
    )
    declare_prefix_tf_frames = DeclareLaunchArgument(
        'prefix_tf_frames',
        default_value='true',
        description=(
            'Prefix local TF frames with robot_name exactly once. Leave false '
            'only for legacy integrations.'
        ),
    )

    return LaunchDescription([
        declare_prefix_tf_frames,
        declare_use_sim_time,
        declare_params_file,
        declare_robot_name,
        declare_namespace_tf,
        declare_tf_robot_pose,
        declare_tf_reference_frame,
        declare_speed_ref,
        declare_convergence_gain,
        declare_add_measurement_noise,
        declare_noise_mean_position,
        declare_noise_stddev_position,
        declare_noise_mean_yaw,
        declare_noise_stddev_yaw,
        OpaqueFunction(function=_launch_setup),
    ])
