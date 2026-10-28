"""Launch the Polaris low-level follower controller."""

import logging
import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

_logger = logging.getLogger('follower_control.launch')

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
    """Prefix robot-local TF frames while preserving global ``sim_world``."""
    if not frame or frame == 'sim_world' or frame == 'world':
        return frame
    if not prefix_tf_frames:
        return frame
    if frame == robot_name or frame.startswith(f'{robot_name}/'):
        return frame
    return f'{robot_name}/{frame}'


def _launch_setup(context, *args, **kwargs):
    """Evaluate launch args in context so float gains are typed correctly."""
    package_name = 'polaris_control'

    use_sim_time = LaunchConfiguration('use_sim_time')
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
    kp_pos = float(LaunchConfiguration('kp_pos').perform(context))
    kp_orient = float(LaunchConfiguration('kp_orient').perform(context))
    tf_robot_pose_arg = LaunchConfiguration('tf_robot_pose').perform(context)
    tf_inertial_link_arg = LaunchConfiguration('tf_inertial_link').perform(context)
    prefix_tf_frames = _as_bool(
        LaunchConfiguration('prefix_tf_frames').perform(context)
    )

    pkg_share = FindPackageShare(package_name).perform(context)
    if len(params_file) == 0:
        raise RuntimeError(
            'params_file launch arg must be set to a YAML file in '
            f'{pkg_share}/config'
        )
    param_config_file = os.path.join(pkg_share, 'config', params_file)

    with open(param_config_file, encoding='utf-8') as parameter_file:
        parameter_config = yaml.safe_load(parameter_file)
    follower_parameters = parameter_config['follower_control']['ros__parameters']

    tf_robot_pose = _resolve_tf_frame(
        robot_name,
        tf_robot_pose_arg or follower_parameters.get('tf_robot_pose', ''),
        prefix_tf_frames,
    )
    tf_inertial_link = _resolve_tf_frame(
        robot_name,
        tf_inertial_link_arg or follower_parameters.get('tf_inertial_link', ''),
        prefix_tf_frames,
    )

    # YAML file sets all defaults; the dict overrides only explicit args so
    # docker-compose / CI can tune them without touching the params file.
    overrides = {
        'kp_pos': kp_pos,
        'kp_orient': kp_orient,
        'use_sim_time': use_sim_time,
        'tf_robot_pose': tf_robot_pose,
        'tf_inertial_link': tf_inertial_link,
        'cmd_vel_topic': 'cmd_vel',
        'vec_to_follow_topic': 'vec_to_follow',
        'orient_point_topic': 'inspection_pose',
        'stop_robot_topic': 'stop_robot',
        'stop_control_topic': 'stop_control',
        'clear_planner_service': 'clear_planner',
        # Docking server output, forwarded only in the EXTERNAL state so that
        # cmd_vel keeps a single publisher.
        'external_cmd_vel_topic': 'cmd_vel_external',
    }
    if follower_parameters.get('pose_topic_type') == 'TFMessage':
        overrides['pose_topic'] = _tf_topic(namespace_tf)

    tf_remappings = _tf_remappings(namespace_tf)

    feedback_node = Node(
        package=package_name,
        executable='follower_control.py',
        name='follower_control',
        namespace=robot_name,
        output='screen',
        parameters=[
            follower_parameters,
            {'use_sim_time': use_sim_time},
            overrides,
        ],
        remappings=tf_remappings,
    )

    return [feedback_node]


def generate_launch_description():
    """Return the configurable Polaris follower launch description."""
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='',
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
        description='Use the simulation clock for the follower node.',
    )

    # Proportional gain for linear velocity in CONTROL_POSITION.
    declare_kp_pos = DeclareLaunchArgument(
        'kp_pos',
        default_value='1.0',
        description=(
            'Proportional gain for linear velocity in CONTROL_POSITION state'
        ),
    )

    # Proportional gain for angular velocity in ALIGN_YAW (P controller)
    declare_kp_orient = DeclareLaunchArgument(
        'kp_orient',
        default_value='1.0',
        description=(
            'Proportional gain for angular velocity in ALIGN_YAW state'
        ),
    )

    # TF child frame identifying the robot body (e.g. pioneer, scout_mini).
    # Empty string means: use whatever is in the params YAML.
    declare_tf_robot_pose = DeclareLaunchArgument(
        'tf_robot_pose',
        default_value='',
        description=(
            'TF child frame for the robot body (e.g. pioneer, scout_mini). '
            'If empty, the value from params_file YAML is used.'
        ),
    )

    # TF parent (inertial/world) frame used by the follower controller.
    # Empty string means: use whatever is in the params YAML.
    declare_tf_inertial_link = DeclareLaunchArgument(
        'tf_inertial_link',
        default_value='',
        description=(
            'TF parent (world/inertial) frame (e.g. map). '
            'If empty, the value from params_file YAML is used.'
        ),
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
        declare_kp_pos,
        declare_kp_orient,
        declare_tf_robot_pose,
        declare_tf_inertial_link,
        OpaqueFunction(function=_launch_setup),
    ])
