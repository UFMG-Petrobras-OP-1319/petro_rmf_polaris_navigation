"""Launch the Polaris planner and vector-field controller."""

import logging
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from nav2_common.launch import RewrittenYaml

_logger = logging.getLogger('navigation.launch')


def _as_bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, *_args, **_kwargs):
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
    tf_robot_pose = LaunchConfiguration('tf_robot_pose').perform(context)
    speed_ref = LaunchConfiguration('speed_ref').perform(context)
    convergence_gain = LaunchConfiguration('convergence_gain').perform(context)
    add_measurement_noise = LaunchConfiguration('add_measurement_noise').perform(context).lower() == 'true'
    noise_mean_position = float(LaunchConfiguration('noise_mean_position').perform(context))
    noise_stddev_position = float(LaunchConfiguration('noise_stddev_position').perform(context))
    noise_mean_yaw = float(LaunchConfiguration('noise_mean_yaw').perform(context))
    noise_stddev_yaw = float(LaunchConfiguration('noise_stddev_yaw').perform(context))
    tf_reference_frame = LaunchConfiguration('tf_reference_frame').perform(
        context
    )

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

    # YAML file sets all defaults; build an optional override dict for frame
    # names so docker-compose / CI can tune them without touching the YAML.
    overrides = {
        'add_measurement_noise': add_measurement_noise,
        'noise_mean_position': noise_mean_position,
        'noise_stddev_position': noise_stddev_position,
        'noise_mean_yaw': noise_mean_yaw,
        'noise_stddev_yaw': noise_stddev_yaw,
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

    if tf_reference_frame:
        overrides['tf_reference_frame'] = tf_reference_frame
    else:
        _logger.warning(
            '⚠️  tf_reference_frame not set via launch arg — '
            'falling back to value in %s. Pass '
            'tf_reference_frame:=<frame> to override.',
            params_file,
        )

    if speed_ref:
        overrides['speed_ref'] = float(speed_ref)
    else:
        _logger.warning(
            '⚠️  speed_ref not set via launch arg — '
            'falling back to value in %s. Pass speed_ref:=<value> to override.',
            params_file,
        )

    if convergence_gain:
        overrides['convergence_gain'] = float(convergence_gain)
    else:
        _logger.warning(
            '⚠️  convergence_gain not set via launch arg — '
            'falling back to value in %s. Pass convergence_gain:=<value> to override.',
            params_file,
        )

    parameters = [namespaced_params, {'use_sim_time': use_sim_time}]
    if overrides:
        parameters.append(overrides)

    controller_node = Node(
        package=package_name,
        executable='vector_field_controller',
        name='controller',
        namespace=node_namespace,
        output='screen',
        parameters=parameters,
        remappings=tf_remappings,
    )

    planner_node = Node(
        package='polaris_planning',
        executable='path_from_points',
        name='planner',
        namespace=node_namespace,
        output='screen',
        parameters=parameters,
        remappings=tf_remappings,
    )

    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_publisher',
        namespace=node_namespace,
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=tf_remappings,
    )

    static_tf_body_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_body_to_livox_frame_publisher',
        namespace=node_namespace,
        # livox is 32 cm above the body frame
        arguments=['0', '0', '0.32', '0', '0', '0', 'body', 'livox_frame'],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=tf_remappings,
    )

    return [
        controller_node,
        planner_node,
        static_tf_map_to_odom,
        static_tf_body_to_livox_frame,
    ]


def generate_launch_description():
    """Return the configurable Polaris navigation launch description."""
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
            'Namespace applied to every Polaris node and relative ROS name.'
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
            'TF parent (world/reference) frame (e.g. sim_world). '
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
        description='Enable Gaussian measurement-noise injection in updateRobotPose (controller node).',
    )

    declare_noise_mean_position = DeclareLaunchArgument(
        'noise_mean_position',
        default_value='0.0',
        description='Mean of the Gaussian noise added to x/y position measurements (m).',
    )

    declare_noise_stddev_position = DeclareLaunchArgument(
        'noise_stddev_position',
        default_value='0.0',
        description='Standard deviation of the Gaussian noise added to x/y position measurements (m).',
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

    return LaunchDescription([
        declare_params_file,
        declare_robot_namespace,
        declare_namespace_tf,
        declare_use_sim_time,
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
