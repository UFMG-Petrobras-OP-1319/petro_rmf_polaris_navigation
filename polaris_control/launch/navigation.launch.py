import logging
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

_logger = logging.getLogger('navigation.launch')


def _robot_name(namespace, name):
    if not namespace:
        return f'/{name}'
    return f'/{namespace.strip("/")}/{name}'


def _launch_setup(context, *_args, **_kwargs):
    package_name = 'polaris_control'

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file').perform(context)
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    tf_robot_pose = LaunchConfiguration('tf_robot_pose').perform(context)
    tf_reference_frame = LaunchConfiguration('tf_reference_frame').perform(context)
    speed_ref = LaunchConfiguration('speed_ref').perform(context)
    convergence_gain = LaunchConfiguration('convergence_gain').perform(context)
    add_measurement_noise = (
        LaunchConfiguration('add_measurement_noise')
        .perform(context)
        .lower() == 'true'
    )
    noise_mean_position = float(LaunchConfiguration('noise_mean_position').perform(context))
    noise_stddev_position = float(LaunchConfiguration('noise_stddev_position').perform(context))
    noise_mean_yaw = float(LaunchConfiguration('noise_mean_yaw').perform(context))
    noise_stddev_yaw = float(LaunchConfiguration('noise_stddev_yaw').perform(context))

    pkg_share = FindPackageShare(package_name).perform(context)
    param_config_file = os.path.join(pkg_share, 'config', params_file)

    with open(param_config_file, encoding='utf-8') as parameter_file:
        parameter_config = yaml.safe_load(parameter_file)
    controller_parameters = parameter_config['controller']['ros__parameters']
    planner_parameters = parameter_config['planner']['ros__parameters']

    # YAML file sets all defaults; build optional override dictionaries for frame
    # names so docker-compose / CI can tune them without touching the YAML.
    controller_overrides = {
        'add_measurement_noise': add_measurement_noise,
        'noise_mean_position': noise_mean_position,
        'noise_stddev_position': noise_stddev_position,
        'noise_mean_yaw': noise_mean_yaw,
        'noise_stddev_yaw': noise_stddev_yaw,
    }
    planner_overrides = {}

    if tf_robot_pose:
        controller_overrides['tf_robot_pose'] = tf_robot_pose
        planner_overrides['tf_robot_pose'] = tf_robot_pose
    else:
        _logger.warning(
            '⚠️  tf_robot_pose not set via launch arg — '
            'falling back to value in %s. Pass tf_robot_pose:=<frame> to override.',
            params_file,
        )

    if tf_reference_frame:
        controller_overrides['tf_reference_frame'] = tf_reference_frame
        planner_overrides['tf_reference_frame'] = tf_reference_frame
    else:
        _logger.warning(
            '⚠️  tf_reference_frame not set via launch arg — '
            'falling back to value in %s. Pass tf_reference_frame:=<frame> to override.',
            params_file,
        )

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

    if robot_namespace:
        controller_overrides.update({
            'path_topic_name': _robot_name(robot_namespace, 'ref_path'),
            'cmd_vel_topic_name': _robot_name(robot_namespace, 'vec_to_follow'),
            'closest_obstacle_topic_name': _robot_name(
                robot_namespace, 'closest_obstacle'
            ),
            'is_path_closed_service_name': _robot_name(
                robot_namespace, 'is_path_closed'
            ),
        })
        planner_overrides.update({
            'path_topic_name': _robot_name(robot_namespace, 'ref_path'),
            'visualization_topic_name': _robot_name(
                robot_namespace, 'visual_path'
            ),
            'clicked_point_topic_name': _robot_name(
                robot_namespace, 'goal_pose'
            ),
            'start_service_name': _robot_name(
                robot_namespace, 'start_planner'
            ),
            'clear_service_name': _robot_name(
                robot_namespace, 'clear_planner'
            ),
            'is_path_closed_service_name': _robot_name(
                robot_namespace, 'is_path_closed'
            ),
            'remove_last_point_service_name': _robot_name(
                robot_namespace, 'remove_last_point'
            ),
        })

    controller_node = Node(
        package=package_name,
        executable='vector_field_controller',
        name='controller',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            controller_parameters,
            {'use_sim_time': use_sim_time},
            controller_overrides,
        ],
    )

    planner_node = Node(
        package='polaris_planning',
        executable='path_from_points',
        name='planner',
        namespace=robot_namespace,
        output='screen',
        parameters=[
            planner_parameters,
            {'use_sim_time': use_sim_time},
            planner_overrides,
        ],
    )

    map_frame = _robot_name(robot_namespace, 'map').lstrip('/')
    odom_frame = _robot_name(robot_namespace, 'odom').lstrip('/')
    body_frame = _robot_name(robot_namespace, 'body').lstrip('/')
    livox_frame = _robot_name(robot_namespace, 'livox_frame').lstrip('/')

    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_publisher',
        namespace=robot_namespace,
        arguments=['0', '0', '0', '0', '0', '0', map_frame, odom_frame],
    )

    static_tf_body_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_body_to_livox_frame_publisher',
        namespace=robot_namespace,
        # livox is 32 cm above the body frame
        arguments=[
            '0', '0', '0.32', '0', '0', '0', body_frame, livox_frame
        ],
    )

    return [
        controller_node,
        planner_node,
        static_tf_map_to_odom,
        static_tf_body_to_livox_frame,
    ]


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

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_robot_namespace,
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
