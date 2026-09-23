import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

params_file = "scout_params.yaml"


def generate_launch_description():

    package_name = 'polaris_control' 

    pkg_share = get_package_share_directory(package_name)
    param_config_file = os.path.join(pkg_share, 'config', params_file)

    feedback_node = Node(
        package=package_name,
        executable='feedback_linearization.py',
        name='feedback_linearization',
        output='screen',
        parameters=[param_config_file]         
    )

    return LaunchDescription([
        feedback_node
    ])