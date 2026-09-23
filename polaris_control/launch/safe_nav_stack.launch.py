import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

params_file = "scout_params.yaml"

def generate_launch_description():

    # ===================================================================
    # Obter caminhos de pacotes e arquivos de configuração
    # ===================================================================
    
    # $(find-pkg-share polaris_control)
    polaris_control_share = get_package_share_directory('polaris_control')
    polaris_planning_share = get_package_share_directory('polaris_planning')
    
    # Caminho para o arquivo RViz
    rviz_config_file = os.path.join(
        polaris_control_share, 'config', 'demo_rviz.rviz'
    )
    
    # Caminho para o arquivo de parâmetros do detector de obstáculos
    # $(find-pkg-share polaris_control)/config/closest_obstacle_detector_params.yaml
    detector_params_file = os.path.join(
        polaris_control_share, 'config', 'closest_obstacle_detector_params.yaml'
    )

    param_controller_file = os.path.join(polaris_control_share, 'config', params_file)
    planner_params_file = os.path.join(polaris_planning_share, 'config', 'path_from_points.yaml')

    # ===================================================================
    # Definições dos Nós
    # ===================================================================

    # --- Nó do RViz ---
    # <node pkg="rviz2" exec="rviz2" name="rviz" ...>
    #     <param name="use_sim_time" value="false"/>
    # </node>
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

    # --- Nó do Controlador (Vector Field) ---
    # <node pkg="polaris_control" exec="vector_field_controller" ...>
    controller_node = Node(
        package='polaris_control',
        executable='vector_field_controller',
        name='controller',
        output='screen',
        parameters=[param_controller_file]
    )

    # --- Nó do Planejador ---
    # <node pkg="polaris_planning" exec="path_from_points" ...>
    planner_node = Node(
        package='polaris_planning',
        executable='path_from_points',
        name='planner',
        output='screen',
        parameters=[planner_params_file]
    )

    # --- PERCEPTION - LaserScan Obstacle Detector ---
    # <node pkg="polaris_control" exec="closest_obstacle_detector" ...>
    #     <param from=".../closest_obstacle_detector_params.yaml"/>
    # </node>
    closest_obstacle_detector_node = Node(
        package='polaris_control',
        executable='closest_obstacle_detector',
        name='closest_obstacle_detector',
        output='screen',
        # É assim que se carrega um arquivo de parâmetros YAML em Python
        parameters=[detector_params_file] 
    )

    # --- Nó do Simulador (Comentado) ---
    # <node pkg="polaris_control" exec="robot_simulator.py" ...>
    # robot_sim_node = Node(
    #     package='polaris_control',
    #     executable='robot_simulator.py',
    #     name='robot_sim',
    #     output='screen'
    # )

    # --- Publicador de TF Estática 1 (scout_mini/base_link -> fast_lio/base_link) ---
    # <node pkg="tf2_ros" exec="static_transform_publisher" ...>
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom_publisher',
        # Note que 'args' no XML é uma string única, 
        # mas 'arguments' no Python é uma lista de strings
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # static_tf_map_to_camera_init = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_to_camera_init_publisher',
    #     # Note que 'args' no XML é uma string única, 
    #     # mas 'arguments' no Python é uma lista de strings
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init']
    # )

    #tf from body to livox_frame
    static_tf_body_to_livox_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_body_to_livox_frame_publisher',
        # Note que 'args' no XML é uma string única, 
        # mas 'arguments' no Python é uma lista de strings
        #livox is 37cm above the body (37cm in z axis)
        arguments=['0', '0', '0.32', '0', '0', '0', 'body', 'livox_frame']
    )

    static_tf_livox_frame_to_camera_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_livox_frame_to_camera_frame_publisher',
        # Note que 'args' no XML é uma string única,
        # mas 'arguments' no Python é uma lista de strings
        # Camera is 32 cm above the Livox frame.
        arguments=['0', '0', '0.32', '0', '0', '0', 'livox_frame', 'camera_link']
    )

    # --- Outros TFs Estáticos (Comentados) ---
    # <node pkg="tf2_ros" ... args="0 0 0 0 0 0 world map" />
    # static_tf_world_to_map = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_world_to_map',
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    # )
    
    # <node pkg="tf2_ros" ... args="0 0 0 0 0 0 base_link fast_lio/base_link" />
    # static_tf_base_to_fastlio = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_base_to_fastlio',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'fast_lio/base_link']
    # )

    # --- Nó 'vector_follower' (Comentado) ---
    # <node pkg="polaris_control" exec="vector_follower_node" ...>
    # vector_follower_node = Node(
    #     package='polaris_control',
    #     executable='vector_follower_node',
    #     name='vector_follower',
    #     output='screen'
    # )

    # ===================================================================
    # Retorna a Descrição do Launch
    # ===================================================================
    return LaunchDescription([
        rviz_node,
        controller_node,
        planner_node,
        closest_obstacle_detector_node,
        static_tf_map_to_odom,
        static_tf_body_to_livox_frame,
        static_tf_livox_frame_to_camera_frame,
        # Descomente as linhas abaixo se quiser adicionar os nós comentados
        # robot_sim_node,
        # static_tf_world_to_map,
        # static_tf_base_to_fastlio,
        # vector_follower_node,
    ])
