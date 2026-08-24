"""Launch the Polaris low-level follower controller."""

import logging
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml

_logger = logging.getLogger('follower_control.launch')


def _as_bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    """Evaluate launch args in context so float gains are typed correctly."""
    package_name = 'polaris_control'

    params_file = LaunchConfiguration('params_file').perform(context)
    robot_namespace = LaunchConfiguration('robot_namespace').perform(
        context
    ).strip('/')
    namespace_tf = _as_bool(
        LaunchConfiguration('namespace_tf').perform(context)
    )
    use_sim_time = _as_bool(
        LaunchConfiguration('use_sim_time').perform(context)
    )
    kp_pos = float(LaunchConfiguration('kp_pos').perform(context))
    kp_orient = float(LaunchConfiguration('kp_orient').perform(context))
    tf_robot_pose = LaunchConfiguration('tf_robot_pose').perform(context)
    tf_inertial_link = LaunchConfiguration('tf_inertial_link').perform(context)

    if namespace_tf and not robot_namespace:
        raise RuntimeError(
            'namespace_tf=true requires a non-empty robot_namespace'
        )

    pkg_share = FindPackageShare(package_name).perform(context)
    param_config_file = os.path.join(pkg_share, 'config', params_file)
    namespaced_params = RewrittenYaml(
        source_file=param_config_file,
        param_rewrites={},
        root_key=robot_namespace or None,
        convert_types=True,
    )

    tf_remappings = []
    if namespace_tf:
        tf_remappings = [
            ('/tf', f'/{robot_namespace}/tf'),
            ('/tf_static', f'/{robot_namespace}/tf_static'),
        ]
    node_namespace = robot_namespace or None

    # YAML file sets all defaults; the dict overrides only explicit args so
    # docker-compose / CI can tune them without touching the params file.
    overrides = {
        'kp_pos': kp_pos,
        'kp_orient': kp_orient,
        'use_sim_time': use_sim_time,
    }

    if tf_robot_pose:
        overrides['tf_robot_pose'] = tf_robot_pose
    else:
        _logger.warning(
            '⚠️  tf_robot_pose not set via launch arg — '
            'falling back to value in %s. Pass tf_robot_pose:=<frame> '
            'to override.',
            params_file,
        )

    if tf_inertial_link:
        overrides['tf_inertial_link'] = tf_inertial_link
    else:
        _logger.warning(
            '⚠️  tf_inertial_link not set via launch arg — '
            'falling back to value in %s. Pass tf_inertial_link:=<frame> '
            'to override.',
            params_file,
        )

    feedback_node = Node(
        package=package_name,
        executable='follower_control.py',
        name='follower_control',
        namespace=node_namespace,
        output='screen',
        parameters=[
            namespaced_params,
            overrides,
        ],
        remappings=tf_remappings,
    )

    return [feedback_node]


def generate_launch_description():
    """Return the configurable Polaris follower launch description."""
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value='pioneer_params.yaml',
        description=(
            'Controller params YAML filename under polaris_control/config/'
        ),
    )

    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description=(
            'Namespace applied to the follower and its relative ROS names.'
        ),
    )

    declare_namespace_tf = DeclareLaunchArgument(
        'namespace_tf',
        default_value='false',
        description=(
            'Remap /tf and /tf_static into robot_namespace. Requires a '
            'non-empty robot_namespace.'
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
            'TF parent (world/inertial) frame (e.g. sim_world). '
            'If empty, the value from params_file YAML is used.'
        ),
    )

    return LaunchDescription([
        declare_params_file,
        declare_robot_namespace,
        declare_namespace_tf,
        declare_use_sim_time,
        declare_kp_pos,
        declare_kp_orient,
        declare_tf_robot_pose,
        declare_tf_inertial_link,
        OpaqueFunction(function=_launch_setup),
    ])
