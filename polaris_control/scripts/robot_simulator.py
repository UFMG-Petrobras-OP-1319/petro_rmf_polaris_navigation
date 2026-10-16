#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, TransformStamped, Point
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster

import numpy as np

class SphereDemo(Node):
    """
    This node is a demonstration of a robot simulator that publishes its position
    as a TF and a Marker in a ROS2 environment. It simulates a robot moving
    in a path based on velocity commands received from a topic.
    """

    def __init__(self):
        super().__init__('circle_tf_broadcaster_node')

        # --- Parâmetros do Robô ---
        self.declare_parameter('parent_frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('marker_topic', 'ball_marker')
        self.declare_parameter('ball_radius', 0.1)

        # --- Parâmetros do Obstáculo ---
        self.declare_parameter('closest_obstacle_topic', 'closest_obstacle')
        self.declare_parameter('visual_obstacle_topic', 'visual_obstacle')
        self.declare_parameter('obstacle_pos', [2.1, 2.1, 0.0])
        self.declare_parameter('obstacle_radius', 0.15)

        # --- Carregar Parâmetros ---
        self.parent_frame_id = self.get_parameter('parent_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        closest_obstacle_topic = self.get_parameter('closest_obstacle_topic').get_parameter_value().string_value
        visual_obstacle_topic = self.get_parameter('visual_obstacle_topic').get_parameter_value().string_value

        # --- Estado do Robô ---
        self.current_pose = np.array([0.5, 0.0, 0.0])
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.last_update_time = self.get_clock().now()

        # --- Estado do Obstáculo ---
        self.obstacle_position = np.array(self.get_parameter('obstacle_pos').get_parameter_value().double_array_value)

        # --- Componentes ROS2 ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.robot_marker_publisher = self.create_publisher(Marker, marker_topic, 10)
        
        # --- Publishers do Obstáculo ---
        self.obstacle_pos_publisher = self.create_publisher(Point, closest_obstacle_topic, 10)
        self.obstacle_marker_publisher = self.create_publisher(Marker, visual_obstacle_topic, 10)

        self.subscription = self.create_subscription(
            Vector3,
            input_topic,
            self.velocity_callback,
            10)

        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.main_loop)

        # --- Logs de Inicialização ---
        self.get_logger().info('Nó Circle TF Broadcaster iniciado.')
        self.get_logger().info(f"Publicando TF de '{self.parent_frame_id}' -> '{self.child_frame_id}'")
        self.get_logger().info(f"Publicando Marker do robô no tópico: '{marker_topic}'")
        self.get_logger().info(f"Subscrevendo ao tópico de velocidade: '{input_topic}'")
        self.get_logger().info(f"Publicando posição do obstáculo em: '{closest_obstacle_topic}'")
        self.get_logger().info(f"Publicando marcador do obstáculo em: '{visual_obstacle_topic}'")


    def velocity_callback(self, msg: Vector3):
        """Callback para atualizar a velocidade do robô."""
        self.current_velocity = np.array([msg.x, msg.y, msg.z])

    def main_loop(self):
        """Loop principal executado pelo timer."""
        self.update_and_publish_robot_pose()
        
        self.publish_obstacle()

    def update_and_publish_robot_pose(self):
        """Calcula a nova pose do robô e publica seu TF e Marker."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        displacement = self.current_velocity * dt
        self.current_pose += displacement
        self.last_update_time = current_time

        self.publish_tf()
        self.publish_ball_marker()

    def publish_tf(self):
        """Cria e publica a mensagem de TransformStamped para o robô."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.current_pose[0]
        t.transform.translation.y = self.current_pose[1]
        t.transform.translation.z = self.current_pose[2]
        t.transform.rotation.w = 1.0 # Sem rotação
        self.tf_broadcaster.sendTransform(t)

    def publish_ball_marker(self):
        """Cria e publica a mensagem Marker para visualizar o robô."""
        marker = Marker()
        marker.header.frame_id = self.parent_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "robot_simulation"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.current_pose[0]
        marker.pose.position.y = self.current_pose[1]
        marker.pose.position.z = self.current_pose[2]
        marker.pose.orientation.w = 1.0
        
        radius = self.get_parameter('ball_radius').get_parameter_value().double_value
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = radius * 2.0
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        self.robot_marker_publisher.publish(marker)

    def publish_obstacle(self):
        """Publica a posição do obstáculo e um marcador para visualizá-lo."""
        # Publica a posição como geometry_msgs/Point
        point_msg = Point()
        point_msg.x = self.obstacle_position[0]
        point_msg.y = self.obstacle_position[1]
        point_msg.z = self.obstacle_position[2]
        self.obstacle_pos_publisher.publish(point_msg)

        # Publica o marcador visual
        marker = Marker()
        marker.header.frame_id = self.parent_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "obstacle_simulation"
        marker.id = 1 # ID diferente do robô
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = self.obstacle_position[0]
        marker.pose.position.y = self.obstacle_position[1]
        marker.pose.position.z = self.obstacle_position[2]
        marker.pose.orientation.w = 1.0

        radius = self.get_parameter('obstacle_radius').get_parameter_value().double_value
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = radius * 2.0

        # Cor azul para o obstáculo
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.lifetime = rclpy.duration.Duration(seconds=0.1).to_msg()
        self.obstacle_marker_publisher.publish(marker)


def main(args=None):
    """Função principal para rodar o nó."""
    rclpy.init(args=args)
    circle_tf_broadcaster_node = SphereDemo()
    try:
        rclpy.spin(circle_tf_broadcaster_node)
    except KeyboardInterrupt:
        pass
    finally:
        circle_tf_broadcaster_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
