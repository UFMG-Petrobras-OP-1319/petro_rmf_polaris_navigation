#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import Vector3, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

from rclpy.duration import Duration
from rclpy.time import Time



# Função auxiliar para normalizar um vetor
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


# Função auxiliar para converter um Quaternion para Yaw (ângulo de rotação em Z)
def quaternion_to_euler_yaw(q_ros):
    x, y, z, w = q_ros.x, q_ros.y, q_ros.z, q_ros.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class VectorFollowerNode(Node):
    def __init__(self):
        super().__init__('vector_follower_node')
        
        # ## Parâmetros ##
        self.declare_parameter('distancia_ponto_controle', 0.15)
        self.declare_parameter('const_vel', 0.3)
        self.declare_parameter('const_omega', 2.0)
        self.declare_parameter('stuck_timeout', 3.0)
        self.declare_parameter('escape_duration', 2.0)
        self.declare_parameter('noise_magnitude', 0.4)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('cmd_vel_topic', "cmd_vel")
        self.declare_parameter('vec_to_follow_topic', "vec_to_follow")
        self.declare_parameter('pose_topic', "Odometry")
        self.declare_parameter('pose_topic_type', "TFMessage")
        self.declare_parameter('tf_robot_pose', "body")
        self.declare_parameter('tf_inertial_link', "camera_init")
        
        self.distancia_ponto_controle = self.get_parameter('distancia_ponto_controle').get_parameter_value().double_value
        self.const_vel = self.get_parameter('const_vel').get_parameter_value().double_value
        self.const_omega = self.get_parameter('const_omega').get_parameter_value().double_value
        self.STUCK_TIMEOUT = self.get_parameter('stuck_timeout').get_parameter_value().double_value
        self.ESCAPE_DURATION = self.get_parameter('escape_duration').get_parameter_value().double_value
        self.NOISE_MAGNITUDE = self.get_parameter('noise_magnitude').get_parameter_value().double_value
        self.timer_period = self.get_parameter('control_period').get_parameter_value().double_value
        if self.timer_period <= 0.0:
            self.get_logger().warn("control_period must be positive; using 0.05s.")
            self.timer_period = 0.05
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.vec_to_follow_topic = self.get_parameter('vec_to_follow_topic').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.pose_topic_type = self.get_parameter('pose_topic_type').get_parameter_value().string_value
        self.tf_robot_pose = self.get_parameter('tf_robot_pose').get_parameter_value().string_value
        self.tf_inertial_link = self.get_parameter('tf_inertial_link').get_parameter_value().string_value

        # ## TF2 Listener ##
        # O Buffer armazena as transformações recebidas e o Listener as preenche.
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ## Publishers e Subscribers ##
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.vector_subscriber = self.create_subscription(Vector3, self.vec_to_follow_topic, self.vector_callback, 10)


        if self.pose_topic_type == "TFMessage":
            self.get_logger().info("Modo: Utilizando TF2 para orientação.")
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        elif self.pose_topic_type == "Odometry":
            self.get_logger().info(f"Modo: Utilizando tópico {self.pose_topic} para orientação.")
            self.pose_subscriber = self.create_subscription(
                Odometry,
                self.pose_topic,
                self.Odometry_callback,
                10)
        elif self.pose_topic_type in ("PoseWithCovarience", "PoseWithCovarianceStamped"):
            self.get_logger().info(
                f"Modo: PoseWithCovarianceStamped em {self.pose_topic} + TF "
                f"({self.tf_inertial_link} <- {self.tf_robot_pose}) para orientação contínua."
            )
            self.pose_subscriber = self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self.amcl_pose_callback,
                10,
            )
            # AMCL often stops publishing while the robot is still; the map->base TF still
            # updates from map->odom × odom->base so we read yaw every control cycle like
            # vector_field_control does.
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        else:
            self.get_logger().error(
                f"pose_topic_type inválido: '{self.pose_topic_type}'. "
                "Use TFMessage, Odometry ou PoseWithCovarianceStamped."
            )

        # ## Variáveis de Estado ##
        self.current_vector = None
        #self.theta = 0.0 # Orientação do robô, será atualizada via TF
        self.theta = None
        self.stuck_timer = 0.0
        self.escape_timer = 0.0
        self.escape_vector = np.array([0.0, 0.0])
        
        # ## Timer do Loop de Controle ##
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info(
            f"Nó feedback_linearization iniciado (pose: {self.pose_topic_type}, period: {self.timer_period:.3f}s)."
        )

    def vector_callback(self, msg):
        """Armazena o vetor de velocidade desejada mais recente."""
        self.current_vector = msg

    def amcl_pose_callback(self, msg):
        """Atualiza yaw a partir de uma PoseWithCovarianceStamped."""
        # Extrai o quaternion da mensagem de pose
        orientation_q = msg.pose.pose.orientation
        # Converte o quaternion para o ângulo yaw (theta) e armazena
        self.theta = quaternion_to_euler_yaw(orientation_q)
        # self.get_logger().info("Entrei no callback...", throttle_duration_sec=5)
    
    def Odometry_callback(self, msg):
        """Atualiza yaw a partir da odometria configurada."""
        orientation_q = msg.pose.pose.orientation
        self.theta = quaternion_to_euler_yaw(orientation_q)

    def _update_theta_from_tf(self) -> bool:
        """Atualiza self.theta a partir do TF mais recente. Retorna True se obteve transform."""
        t = Time()
        try:
            if not self.tf_buffer.can_transform(
                self.tf_inertial_link,
                self.tf_robot_pose,
                t,
                timeout=Duration(seconds=0.05),
            ):
                return False
            trans = self.tf_buffer.lookup_transform(
                self.tf_inertial_link,
                self.tf_robot_pose,
                t,
            )
            self.theta = quaternion_to_euler_yaw(trans.transform.rotation)
            return True
        except TransformException:
            return False

    def control_loop(self):
        """Loop principal que calcula e publica os comandos de velocidade."""
        if self.current_vector is None:
            self.get_logger().info(
                f"Aguardando vetor no tópico {self.vec_to_follow_topic}...",
                throttle_duration_sec=5,
            )
            return

        if self.pose_topic_type == "TFMessage":
            if not self._update_theta_from_tf():
                self.get_logger().warn(
                    f"Waiting TF: {self.tf_inertial_link} <- {self.tf_robot_pose}",
                    throttle_duration_sec=2,
                )
                return

        elif self.pose_topic_type in ("PoseWithCovarience", "PoseWithCovarianceStamped"):
            # TF em cada tick; a pose só preenche theta até o TF estar pronto.
            self._update_theta_from_tf()

        if self.theta is None:
            self.get_logger().info(
                "Aguardando orientação (TF / pose / odometry)...",
                throttle_duration_sec=5,
            )
            return

        # 1. Obter a transformação de 'odom' para 'base_footprint'
        # try:
        #     # Pede a transformação mais recente disponível
        #     trans = self.tf_buffer.lookup_transform('world', 'scout_mini/base_link', rclpy.time.Time())
        # except TransformException as ex:
        #     self.get_logger().warn(f'Não foi possível obter a transformação: {ex}', throttle_duration_sec=2)
        #     return

        # Extrai a rotação (quaternion) e a converte para yaw (theta)
        #self.theta = quaternion_to_euler_yaw(trans.transform.rotation)
        
        dt = self.timer_period
        
        # 2. Obter o vetor de velocidade desejada e aplicar manobra de escape se ativa
        x_dot = self.current_vector.x
        y_dot = self.current_vector.y
        psi_des = np.array([x_dot, y_dot])
        
        if self.escape_timer > 0.0:
            psi_des += self.escape_vector
            self.escape_timer = max(0.0, self.escape_timer - dt)
            if self.escape_timer == 0.0:
                self.get_logger().info("Manobra de escape concluída.")
        
        # 3. Transformar o vetor de velocidade (frame do robô) em comandos V e w
        # Usando a conversão simplificada onde (x_dot, y_dot) já são as velocidades
        # desejadas no frame do robô.
        # V_final = psi_des[0]
        # w_final = psi_des[1] / self.distancia_ponto_controle

        # Lógica CORRETA para esta arquitetura
        c, s = np.cos(self.theta), np.sin(self.theta)
        x_dot_global = psi_des[0]
        y_dot_global = psi_des[1]

        # Esta é a rotação que transforma o vetor global em comandos locais
        V_final = x_dot_global * c + y_dot_global * s
        w_final = (1 / self.distancia_ponto_controle) * (-x_dot_global * s + y_dot_global * c)
        
        # 4. Lógica de Detecção de Robô Preso
        velocidade_desejada_significativa = np.linalg.norm(psi_des) > 0.1
        velocidade_final_nula = abs(V_final) < 0.05
        
        if velocidade_desejada_significativa and velocidade_final_nula and self.escape_timer == 0.0:
            self.stuck_timer += dt
        else:
            self.stuck_timer = 0.0

        # if self.stuck_timer > self.STUCK_TIMEOUT and self.escape_timer == 0.0:
        #     self.get_logger().warn(f"Robô preso! INICIANDO MANOBRA DE ESCAPE.")
        #     self.stuck_timer = 0.0
        #     self.escape_timer = self.ESCAPE_DURATION
        #     random_vec = np.random.randn(2)
        #     self.escape_vector = normalize(random_vec) * self.NOISE_MAGNITUDE

        # 5. Limitar as velocidades
        V_final = np.clip(V_final, -self.const_vel, self.const_vel)
        w_final = np.clip(w_final, -self.const_omega, self.const_omega)
        
        # 6. Publicar o comando de velocidade
        twist_msg = Twist()
        twist_msg.linear.x = V_final
        twist_msg.angular.z = w_final
        self.cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VectorFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_vel_publisher.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
