#!/usr/bin/env python3

import math

from FollowerControlState import FollowerControlState
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def quaternion_to_euler_yaw(q_ros):
    x, y, z, w = q_ros.x, q_ros.y, q_ros.z, q_ros.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)


class VectorFollowerNode(Node):
    def __init__(self):
        super().__init__('vector_follower_node')

        # ## Parameters ##
        self.declare_parameter('distancia_ponto_controle', 0.15)
        self.declare_parameter('const_vel', 0.3)
        self.declare_parameter('const_omega', 2.0)
        self.declare_parameter('stuck_timeout', 3.0)
        self.declare_parameter('escape_duration', 2.0)
        self.declare_parameter('noise_magnitude', 0.4)
        self.declare_parameter('control_period', 0.05)
        self.declare_parameter('cmd_vel_topic', "/cmd_vel")
        self.declare_parameter('vec_to_follow_topic', "/vec_to_follow")
        self.declare_parameter('pose_topic', "/Odometry")
        self.declare_parameter('pose_topic_type', "TFMessage")
        self.declare_parameter('tf_robot_pose', "body")
        self.declare_parameter('tf_inertial_link', "camera_init")
        self.declare_parameter('orient_point_topic', "/orient_target")
        self.declare_parameter('kp_orient', 1.0)
        self.declare_parameter('kp_pos', 1.0)
        self.declare_parameter('stop_robot_topic', "/stop_robot")
        self.declare_parameter('stop_control_topic', "/stop_control")
        self.declare_parameter('clear_planner_service', "/clear_planner")

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
        self.orient_point_topic = self.get_parameter('orient_point_topic').get_parameter_value().string_value
        self.kp_orient = self.get_parameter('kp_orient').get_parameter_value().double_value
        self.kp_pos = self.get_parameter('kp_pos').get_parameter_value().double_value
        self.stop_robot_topic = self.get_parameter('stop_robot_topic').get_parameter_value().string_value
        self.stop_control_topic = self.get_parameter('stop_control_topic').get_parameter_value().string_value
        self.clear_planner_service_name = self.get_parameter('clear_planner_service').get_parameter_value().string_value

        # ## TF2 Listener ##
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
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        else:
            self.get_logger().error(
                f"pose_topic_type inválido: '{self.pose_topic_type}'. "
                "Use TFMessage, Odometry ou PoseWithCovarianceStamped."
            )

        # ## Publishers ##
        self.cmd_vel_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # ## Subscribers ##
        self.vector_subscriber = self.create_subscription(
            Vector3, self.vec_to_follow_topic, self.vector_callback, 10)
        # inspection_pose and stop_control are one-shot values published by the
        # fleet manager. TRANSIENT_LOCAL lets this node recover the latest target
        # and desired control state after discovery races or reconnects.
        one_shot_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseStamped, self.orient_point_topic, self._orient_target_cb, one_shot_qos)
        self.create_subscription(String, self.stop_robot_topic, self._stop_robot_cb, 10)
        self.create_subscription(
            String, self.stop_control_topic, self._stop_control_cb, one_shot_qos)

        # ## Service client for clearing the planner ##
        self.clear_planner_client = self.create_client(Trigger, self.clear_planner_service_name)

        # ## FSM state ##
        self.state = FollowerControlState.STOPPED
        self._stop_robot_flag = False
        self._requested_control_state = None
        self.control_state_rx_count = 0
        self.control_state_applied_count = 0

        # ## State variables ##
        self.current_vector = None
        self.theta = None
        self.stuck_timer = 0.0
        self.escape_timer = 0.0
        self.escape_vector = np.array([0.0, 0.0])
        self.orient_target = None
        self.robot_x = None
        self.robot_y = None

        # ## Control timer ##
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.get_logger().info(
            f"VectorFollowerNode started | pose: {self.pose_topic_type} | "
            f"period: {self.timer_period:.3f}s | kp_pos: {self.kp_pos} | "
            f"kp_orient: {self.kp_orient} | initial state: {self.state.value}"
        )

    # ------------------------------------------------------------------ #
    #  Topic callbacks                                                     #
    # ------------------------------------------------------------------ #

    def vector_callback(self, msg):
        """Store the latest desired velocity vector and trigger STOPPED→CONTROL_POSITION."""
        self.current_vector = msg
        if self.state == FollowerControlState.STOPPED:
            self.state = FollowerControlState.CONTROL_POSITION
            self.get_logger().info("FSM: STOPPED → CONTROL_POSITION")

    def _stop_robot_cb(self, msg: Bool):
        """Set the stop-robot flag; transition is processed in the control loop."""
        if msg.data:
            self._stop_robot_flag = True

    def _stop_control_cb(self, msg: String):
        """Store the latest explicit, idempotent follower control state."""
        command = msg.data.strip()
        valid_commands = {
            FollowerControlState.CONTROL_POSITION.value,
            FollowerControlState.ALIGN_YAW.value,
        }
        if command not in valid_commands:
            self.get_logger().warn(
                f"Ignoring invalid control state '{msg.data}' on "
                f"{self.stop_control_topic}; expected CONTROL_POSITION or ALIGN_YAW."
            )
            return

        requested_state = FollowerControlState(command)
        pending_before = self._requested_control_state
        self._requested_control_state = requested_state
        self.control_state_rx_count += 1
        self.get_logger().info(
            "[FOLLOWER_CONTROL_STATE_RX] "
            f"rx_count={self.control_state_rx_count} "
            f"requested={requested_state.value} "
            f"pending_before={pending_before.value if pending_before else 'NONE'} "
            f"current={self.state.value}"
        )

    def _orient_target_cb(self, msg: PoseStamped):
        """Store the target point the robot should face in ALIGN_YAW state."""
        self.orient_target = msg.pose.position

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        orientation_q = msg.pose.pose.orientation
        self.theta = quaternion_to_euler_yaw(orientation_q)
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def Odometry_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        self.theta = quaternion_to_euler_yaw(orientation_q)
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def _update_theta_from_tf(self) -> bool:
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
            self.robot_x = trans.transform.translation.x
            self.robot_y = trans.transform.translation.y
            return True
        except TransformException:
            return False

    # ------------------------------------------------------------------ #
    #  FSM helper                                                          #
    # ------------------------------------------------------------------ #

    def _do_stop_robot(self):
        """Transition to STOPPED: halt robot and clear all references.

        CONTROL_POSITION → STOPPED: also calls clear_planner (discards the path reference).
        ALIGN_YAW → STOPPED: only clears the yaw reference, no planner interaction.
        """
        prev_state = self.state
        self.state = FollowerControlState.STOPPED
        self.current_vector = None
        self.orient_target = None
        self.cmd_vel_publisher.publish(Twist())
        if prev_state == FollowerControlState.CONTROL_POSITION:
            if self.clear_planner_client.service_is_ready():
                self.clear_planner_client.call_async(Trigger.Request())
            else:
                self.get_logger().warn(
                    f"clear_planner service '{self.clear_planner_service_name}' not available; skipping."
                )
        self.get_logger().info(f"FSM: {prev_state.value} → STOPPED (stop_robot received)")

    # ------------------------------------------------------------------ #
    #  Control loop                                                        #
    # ------------------------------------------------------------------ #

    def control_loop(self):
        # --- Update pose from TF when applicable ---
        if self.pose_topic_type == "TFMessage":
            if not self._update_theta_from_tf():
                self.get_logger().warn(
                    f"Waiting TF: {self.tf_inertial_link} <- {self.tf_robot_pose}",
                    throttle_duration_sec=2,
                )
                return
        elif self.pose_topic_type in ("PoseWithCovarience", "PoseWithCovarianceStamped"):
            self._update_theta_from_tf()

        if self.theta is None:
            self.get_logger().info(
                "Aguardando orientação (TF / pose / odometry)...",
                throttle_duration_sec=5,
            )
            return

        # --- FSM transitions (consume requests, then reset) ---
        if self._stop_robot_flag:
            self._stop_robot_flag = False
            self._do_stop_robot()

        # STOPPED remains controlled by /stop_robot and vector availability. Keep
        # the latest requested mode pending until the follower is active again.
        if (
            self._requested_control_state is not None
            and self.state != FollowerControlState.STOPPED
        ):
            requested_state = self._requested_control_state
            self._requested_control_state = None
            previous_state = self.state

            if (
                requested_state == FollowerControlState.ALIGN_YAW
                and self.state != FollowerControlState.ALIGN_YAW
            ):
                self.state = requested_state
                self.current_vector = None
                if self.clear_planner_client.service_is_ready():
                    self.clear_planner_client.call_async(Trigger.Request())
                else:
                    self.get_logger().warn(
                        f"clear_planner service '{self.clear_planner_service_name}' not available; skipping."
                    )
            elif requested_state == FollowerControlState.CONTROL_POSITION:
                self.state = requested_state

            self.control_state_applied_count += 1
            self.get_logger().info(
                "[FOLLOWER_CONTROL_STATE_APPLIED] "
                f"applied_count={self.control_state_applied_count} "
                f"requested={requested_state.value} "
                f"state_before={previous_state.value} state_after={self.state.value} "
                f"transitioned={previous_state != self.state}"
            )

        # --- Dispatch per state ---
        if self.state == FollowerControlState.STOPPED:
            self.cmd_vel_publisher.publish(Twist())
            return

        elif self.state == FollowerControlState.ALIGN_YAW:
            self._run_align_yaw()

        elif self.state == FollowerControlState.CONTROL_POSITION:
            self._run_control_position()

    # ------------------------------------------------------------------ #
    #  State behaviours                                                    #
    # ------------------------------------------------------------------ #

    def _run_align_yaw(self):
        """P controller that rotates the robot to face orient_target."""
        if self.orient_target is None or self.robot_x is None:
            self.get_logger().info(
                "ALIGN_YAW: aguardando orient_target e posição do robô...",
                throttle_duration_sec=5,
            )
            self.cmd_vel_publisher.publish(Twist())
            return
        dx = self.orient_target.x - self.robot_x
        dy = self.orient_target.y - self.robot_y
        theta_des = math.atan2(dy, dx)
        e_theta = math.atan2(
            math.sin(theta_des - self.theta),
            math.cos(theta_des - self.theta),
        )
        w = np.clip(self.kp_orient * e_theta, -self.const_omega, self.const_omega)
        twist_msg = Twist()
        twist_msg.angular.z = w
        self.cmd_vel_publisher.publish(twist_msg)

    def _run_control_position(self):
        """Feedback-linearization vector-follow controller."""
        if self.current_vector is None:
            self.get_logger().info(
                f"CONTROL_POSITION: aguardando vetor no tópico {self.vec_to_follow_topic}...",
                throttle_duration_sec=5,
            )
            return

        dt = self.timer_period

        x_dot = self.current_vector.x
        y_dot = self.current_vector.y
        psi_des = np.array([x_dot, y_dot])

        if self.escape_timer > 0.0:
            psi_des += self.escape_vector
            self.escape_timer = max(0.0, self.escape_timer - dt)
            if self.escape_timer == 0.0:
                self.get_logger().info("Manobra de escape concluída.")

        c, s = np.cos(self.theta), np.sin(self.theta)
        x_dot_global = psi_des[0]
        y_dot_global = psi_des[1]

        # kp_pos scales the linear velocity command from the feedback-linearization law
        V_final = self.kp_pos * (x_dot_global * c + y_dot_global * s)
        w_final = (1 / self.distancia_ponto_controle) * (-x_dot_global * s + y_dot_global * c)

        velocidade_desejada_significativa = np.linalg.norm(psi_des) > 0.1
        velocidade_final_nula = abs(V_final) < 0.05

        if velocidade_desejada_significativa and velocidade_final_nula and self.escape_timer == 0.0:
            self.stuck_timer += dt
        else:
            self.stuck_timer = 0.0

        V_final = np.clip(V_final, -self.const_vel, self.const_vel)
        w_final = np.clip(w_final, -self.const_omega, self.const_omega)

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
        node.cmd_vel_publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
