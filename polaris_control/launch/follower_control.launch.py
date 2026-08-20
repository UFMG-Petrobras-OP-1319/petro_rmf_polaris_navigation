import logging
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

_logger = logging.getLogger('follower_control.launch')


def _robot_name(namespace, name):
    if not namespace:
        return f'/{name}'
    return f'/{namespace.strip("/")}/{name}'


def _launch_setup(context, *args, **kwargs):
    """Evaluate launch args in context so float gains are typed correctly."""
    package_name = 'polaris_control'

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file').perform(context)
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    kp_pos = float(LaunchConfiguration('kp_pos').perform(context))
    kp_orient = float(LaunchConfiguration('kp_orient').perform(context))
    tf_robot_pose = LaunchConfiguration('tf_robot_pose').perform(context)
    tf_inertial_link = LaunchConfiguration('tf_inertial_link').perform(context)

    pkg_share = FindPackageShare(package_name).perform(context)
    param_config_file = os.path.join(pkg_share, 'config', params_file)
    with open(param_config_file, encoding='utf-8') as parameter_file:
        parameter_config = yaml.safe_load(parameter_file)
    follower_parameters = parameter_config['follower_control']['ros__parameters']

    # YAML file sets all defaults; the dict overrides only explicit args so
    # docker-compose / CI can tune them without touching the params file.
    overrides = {'kp_pos': kp_pos, 'kp_orient': kp_orient}

    if tf_robot_pose:
        overrides['tf_robot_pose'] = tf_robot_pose
    else:
        _logger.warning(
            '⚠️  tf_robot_pose not set via launch arg — '
            'falling back to value in %s. Pass tf_robot_pose:=<frame> to override.',
            params_file,
        )

    if tf_inertial_link:
        overrides['tf_inertial_link'] = tf_inertial_link
    else:
        _logger.warning(
            '⚠️  tf_inertial_link not set via launch arg — '
            'falling back to value in %s. Pass tf_inertial_link:=<frame> to override.',
            params_file,
        )

    if robot_namespace:
        overrides.update({
            'cmd_vel_topic': _robot_name(robot_namespace, 'cmd_vel'),
            'vec_to_follow_topic': _robot_name(
                robot_namespace, 'vec_to_follow'
            ),
            'orient_point_topic': _robot_name(
                robot_namespace, 'inspection_pose'
            ),
            'stop_robot_topic': _robot_name(robot_namespace, 'stop_robot'),
            'stop_control_topic': _robot_name(robot_namespace, 'stop_control'),
            'clear_planner_service': _robot_name(
                robot_namespace, 'clear_planner'
            ),
        })

    feedback_node = Node(
        package=package_name,
        executable='follower_control.py',
        name='follower_control',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            follower_parameters,
            {'use_sim_time': use_sim_time},
            overrides,
        ],
    )

    return [feedback_node]


def generate_launch_description():

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the simulation clock.',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='pioneer_params.yaml',
        description='Controller params YAML filename under polaris_control/config/',
    )

    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Robot namespace used when multiple stacks share a domain.',
    )

    # Proportional gain for linear velocity in CONTROL_POSITION (feedback-linearization)
    declare_kp_pos = DeclareLaunchArgument(
        'kp_pos',
        default_value='1.0',
        description='Proportional gain for linear velocity in CONTROL_POSITION state',
    )

    # Proportional gain for angular velocity in ALIGN_YAW (P controller)
    declare_kp_orient = DeclareLaunchArgument(
        'kp_orient',
        default_value='1.0',
        description='Proportional gain for angular velocity in ALIGN_YAW state',
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
            'TF parent (world/inertial) frame (e.g. sim_world). '
            'If empty, the value from params_file YAML is used.'
        ),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_robot_namespace,
        declare_kp_pos,
        declare_kp_orient,
        declare_tf_robot_pose,
        declare_tf_inertial_link,
        OpaqueFunction(function=_launch_setup),
    ])
