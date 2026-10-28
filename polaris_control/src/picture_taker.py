#!/usr/bin/env python3
"""
Take inspection photos on request from the fleet.

The fleet manager publishes a JSON request on take_picture_request and waits for the JSON
result on take_picture_result (picture_rules.py has the contract). This node checks that
the robot is ready, asks the camera for a photo through its TakePhoto service, and sends
the photo back:

  1. the base is still: it stayed within still_linear_m / still_angular_rad for
     still_window_sec (pose from the same TF frames the follower uses);
  2. for a pan_tilt camera, when the request says where the gimbal was aimed: both
     joints are within tolerance_rad of expected_joints (pan modulo 2*pi), with fresh
     feedback, and stayed there for settle_sec;
  3. the camera's take_photo service answers with a frame captured after 1 and 2 held.

Anything that keeps the photo from being taken before ready_timeout_sec is reported back
as a failure with the reason, so the fleet can fail the task.

Cameras are configured by the name the fleet uses (default, inspection, ...):
  cameras: [default, inspection]
  default:    {type: fixed, service: fixed_camera_snapshot/take_photo}
  inspection: {type: pan_tilt, service: hk_camera_rtsp_pub/take_photo,
               joint_state_topic: hk_camera_ptz/joint_states, ...}
"""

from collections import deque
import math
import os
import re

from hk_camera_interfaces.srv import TakePhoto
import picture_rules as rules
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros
from tf2_ros import TransformException

CAMERA_KEYS = (
    ('type', ''),
    ('service', ''),
    ('joint_state_topic', ''),
    ('pan_joint', 'pan_joint'),
    ('tilt_joint', 'tilt_joint'),
    ('tolerance_rad', 0.05),
    ('settle_sec', 1.0),
    ('feedback_max_age_sec', 1.0),
)

WAITING = 'waiting'
CAPTURING = 'capturing'


def _yaw(rotation) -> float:
    siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Job:
    """The request being served."""

    def __init__(self, request, camera, started, deadline):
        self.request = request
        self.camera = camera
        self.started = started
        self.deadline = deadline
        self.phase = WAITING
        self.gimbal_window = rules.StableWindow()
        self.reasons = []
        self.future = None
        self.capture_deadline = None
        self.ready_since = None
        self.retry_at = 0.0
        self.last_camera_error = ''


class PictureTaker(Node):
    """Serve the fleet's photo requests through the robot's camera services."""

    def __init__(self):
        super().__init__('picture_taker')
        self.request_topic = self.declare_parameter(
            'request_topic', 'take_picture_request').value
        self.result_topic = self.declare_parameter(
            'result_topic', 'take_picture_result').value
        self.ready_timeout = float(self.declare_parameter('ready_timeout_sec', 15.0).value)
        self.service_timeout = float(self.declare_parameter('service_timeout_sec', 5.0).value)
        self.still_window = float(self.declare_parameter('still_window_sec', 1.0).value)
        self.still_linear = float(self.declare_parameter('still_linear_m', 0.02).value)
        self.still_angular = float(self.declare_parameter('still_angular_rad', 0.02).value)
        self.pose_max_age = float(self.declare_parameter('pose_max_age_sec', 1.0).value)
        self.photo_dir = self.declare_parameter('photo_dir', '/tmp/polaris_pictures').value
        # 0 sends the photo at the camera's full resolution
        self.max_width = int(self.declare_parameter('max_width', 0).value)
        self.jpeg_quality = int(self.declare_parameter('jpeg_quality', 90).value)
        self.keep_photos = bool(self.declare_parameter('keep_photos', False).value)
        self.tf_robot_pose = self.declare_parameter('tf_robot_pose', 'base_link').value
        self.tf_inertial_link = self.declare_parameter('tf_inertial_link', 'map').value

        self.cameras = {}
        self.camera_errors = {}
        for name in self.declare_parameter('cameras', ['default']).value:
            values = {
                key: self.declare_parameter(f'{name}.{key}', default).value
                for key, default in CAMERA_KEYS
            }
            try:
                self.cameras[name] = rules.parse_camera(name, values)
            except ValueError as error:
                self.camera_errors[name] = str(error)
                self.get_logger().error(f'Unusable camera: {error}')

        os.makedirs(self.photo_dir, exist_ok=True)

        request_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        result_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.result_publisher = self.create_publisher(String, self.result_topic, result_qos)
        self.create_subscription(String, self.request_topic, self._on_request, request_qos)

        self.clients_by_camera = {}
        self.joint_states = {}
        for camera in self.cameras.values():
            self.clients_by_camera[camera.name] = self.create_client(TakePhoto, camera.service)
            if camera.kind == rules.PAN_TILT and camera.joint_state_topic not in self.joint_states:
                self.joint_states[camera.joint_state_topic] = None
                self.create_subscription(
                    JointState,
                    camera.joint_state_topic,
                    lambda msg, topic=camera.joint_state_topic: self._on_joint_state(topic, msg),
                    qos_profile_sensor_data,
                )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.stillness = rules.StillnessTracker(
            self.still_linear, self.still_angular, self.still_window)
        self.pose = None
        self.pose_time = None

        self.seen_requests = deque(maxlen=64)
        self.job = None
        self.last_wait_log = 0.0
        self.create_timer(0.05, self._tick)

        described = ', '.join(
            f'{c.name}: {c.kind} via {self.resolve_service_name(c.service)}'
            + (f' + {self.resolve_topic_name(c.joint_state_topic)}' if c.kind == rules.PAN_TILT
               else '')
            for c in self.cameras.values()
        )
        self.get_logger().info(
            f'Taking photos on {self.resolve_topic_name(self.request_topic)} -> '
            f'{self.resolve_topic_name(self.result_topic)}; cameras: {described or "none"}; '
            f'still when within {self.still_linear} m / {self.still_angular} rad for '
            f'{self.still_window}s ({self.tf_inertial_link} <- {self.tf_robot_pose}); '
            f'max_width {self.max_width or "full resolution"}'
        )

    # ------------------------------------------------------------------ #
    #  Inputs                                                              #
    # ------------------------------------------------------------------ #

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_joint_state(self, topic, msg):
        self.joint_states[topic] = (list(msg.name), list(msg.position), self._now())

    def _measured_joints(self, camera):
        """Joints of a pan_tilt camera and how old they are, or (None, None)."""
        stored = self.joint_states.get(camera.joint_state_topic)
        if stored is None:
            return None, None
        names, positions, received = stored
        joints = rules.joint_positions(names, positions, camera.pan_joint, camera.tilt_joint)
        return joints, self._now() - received

    def _update_pose(self, now):
        try:
            if not self.tf_buffer.can_transform(
                self.tf_inertial_link, self.tf_robot_pose, Time(),
                timeout=Duration(seconds=0.0),
            ):
                return
            transform = self.tf_buffer.lookup_transform(
                self.tf_inertial_link, self.tf_robot_pose, Time())
        except TransformException:
            return
        translation = transform.transform.translation
        yaw = _yaw(transform.transform.rotation)
        self.pose = {'x': translation.x, 'y': translation.y, 'yaw': yaw}
        self.pose_time = now
        self.stillness.update(translation.x, translation.y, yaw, now)

    def _on_request(self, msg):
        try:
            request = rules.parse_request(msg.data)
        except ValueError as error:
            request_id = rules.request_id_of(msg.data)
            self.get_logger().error(f'Invalid photo request: {error}')
            if request_id is not None:
                self._publish(rules.failed(request_id, None, f'invalid request: {error}'))
            return

        # The request topic is transient-local: a reconnect replays the last request
        if request.request_id in self.seen_requests:
            return
        self.seen_requests.append(request.request_id)

        if self.job is not None:
            self._finish_failed(f'superseded by request {request.request_id}')

        camera = self.cameras.get(request.camera)
        if camera is None:
            reason = self.camera_errors.get(
                request.camera,
                f'this robot has no camera {request.camera!r} '
                f'(it has: {", ".join(self.cameras) or "none"})',
            )
            self.get_logger().error(f'Photo {request.request_id} refused: {reason}')
            self._publish(rules.failed(request.request_id, request.camera, reason))
            return

        now = self._now()
        timeout = self.ready_timeout
        if request.timeout_sec is not None:
            timeout = min(timeout, request.timeout_sec)
        self.job = Job(request, camera, now, now + timeout)
        self._publish(rules.accepted(request.request_id, camera.name))
        aim = (
            f'gimbal expected at pan {request.expected_joints["pan"]:.3f}, '
            f'tilt {request.expected_joints["tilt"]:.3f} rad'
            if request.expected_joints and camera.kind == rules.PAN_TILT
            else 'no gimbal check'
        )
        self.get_logger().info(
            f'Photo {request.request_id} accepted: camera {camera.name} ({camera.kind}), '
            f'{aim}, ready within {timeout:.1f}s'
        )

    # ------------------------------------------------------------------ #
    #  Serving the request                                                 #
    # ------------------------------------------------------------------ #

    def _tick(self):
        now = self._now()
        self._update_pose(now)
        if self.job is None:
            return
        if self.job.phase == WAITING:
            self._wait_until_ready(now)
        else:
            self._check_capture(now)

    def _readiness(self, now):
        """Return (when everything became ready, or None, and what is still missing)."""
        job = self.job
        camera = job.camera
        reasons = []

        still_since = None
        if self.pose_time is None or now - self.pose_time > self.pose_max_age:
            reasons.append(
                f'no robot pose ({self.tf_inertial_link} <- {self.tf_robot_pose})')
        else:
            still_since = self.stillness.still_since(now)
            if still_since is None:
                reasons.append('the base is moving')

        ready_since = still_since
        expected = job.request.expected_joints
        if camera.kind == rules.PAN_TILT and expected is not None:
            joints, age = self._measured_joints(camera)
            if joints is None:
                job.gimbal_window.update(False, now)
                reasons.append(
                    f'no {camera.pan_joint}/{camera.tilt_joint} feedback on '
                    f'{camera.joint_state_topic}')
            elif age > camera.feedback_max_age_sec:
                job.gimbal_window.update(False, now)
                reasons.append(f'gimbal feedback is {age:.1f}s old')
            else:
                pan_error, tilt_error = rules.gimbal_errors(expected, joints)
                within = pan_error <= camera.tolerance_rad and tilt_error <= camera.tolerance_rad
                job.gimbal_window.update(within, now)
                if not within:
                    reasons.append(
                        f'gimbal off target: pan error {pan_error:.3f} rad, tilt error '
                        f'{tilt_error:.3f} rad (tolerance {camera.tolerance_rad})')
                elif job.gimbal_window.held_for(now) < camera.settle_sec:
                    reasons.append('gimbal settling')
                elif ready_since is not None:
                    ready_since = max(ready_since, job.gimbal_window.since())

        if not self.clients_by_camera[camera.name].service_is_ready():
            service = self.resolve_service_name(camera.service)
            reasons.append(f'camera service {service} is not available')

        return (None if reasons else ready_since), reasons

    def _wait_until_ready(self, now):
        job = self.job
        if now < job.retry_at:
            return
        ready_since, reasons = self._readiness(now)
        if ready_since is not None:
            self._start_capture(now, ready_since)
            return
        job.reasons = reasons
        if now > job.deadline:
            detail = '; '.join(reasons)
            if job.last_camera_error:
                detail = f'{detail}; last camera error: {job.last_camera_error}'
            self._finish_failed(f'not ready after {now - job.started:.1f}s: {detail}')
            return
        if now - self.last_wait_log > 2.0:
            self.last_wait_log = now
            self.get_logger().info(
                f'Photo {job.request.request_id} waiting: {"; ".join(reasons)}')

    def _photo_path(self):
        safe = re.sub(r'[^A-Za-z0-9_.-]', '_', self.job.request.request_id)
        return os.path.join(self.photo_dir, f'{safe}.jpg')

    def _start_capture(self, now, ready_since):
        job = self.job
        job.ready_since = ready_since
        job.phase = CAPTURING
        job.capture_deadline = now + self.service_timeout
        job.future = self.clients_by_camera[job.camera.name].call_async(
            TakePhoto.Request(save_path=self._photo_path()))

    def _retry_capture(self, now, error):
        """Go back to waiting; the deadline still bounds how long that lasts."""
        job = self.job
        job.last_camera_error = error
        job.phase = WAITING
        job.future = None
        job.retry_at = now + 0.2

    def _check_capture(self, now):
        job = self.job
        if not job.future.done():
            if now > job.capture_deadline:
                self._finish_failed(
                    f'camera service {self.resolve_service_name(job.camera.service)} did not '
                    f'answer within {self.service_timeout:.1f}s')
            return

        try:
            response = job.future.result()
        except Exception as error:  # noqa: B902  a failed call surfaces as any exception
            response = None
            message = str(error)
        else:
            message = response.message if response is not None else 'no response'

        if response is None or not response.success:
            if now > job.deadline:
                self._finish_failed(f'the camera could not take the photo: {message}')
            else:
                self.get_logger().warn(f'Photo {job.request.request_id}: {message}; retrying')
                self._retry_capture(now, message)
            return

        stamp = response.stamp.sec + response.stamp.nanosec / 1e9
        # A frame captured before the robot was ready shows it moving; the next one will do
        if stamp > 0.0 and stamp < job.ready_since - 0.05:
            if now > job.deadline:
                self._finish_failed('the camera only returned frames older than the moment '
                                    'the robot became ready')
            else:
                self._retry_capture(now, 'frame older than the ready moment')
            return

        path = self._photo_path()
        try:
            with open(path, 'rb') as photo:
                data = photo.read()
            jpeg, width, height = rules.encode_photo(data, self.max_width, self.jpeg_quality)
        except (OSError, ValueError) as error:
            self._finish_failed(f'could not read the photo {path}: {error}')
            return
        finally:
            if not self.keep_photos and os.path.exists(path):
                os.remove(path)

        joints = None
        if job.camera.kind == rules.PAN_TILT:
            joints, _ = self._measured_joints(job.camera)
        request_id = job.request.request_id
        self._publish(rules.succeeded(
            request_id, job.camera.name, jpeg, width, height,
            stamp if stamp > 0.0 else now, joints, self.pose,
        ))
        self.get_logger().info(
            f'Photo {request_id} sent: {width}x{height}, {len(jpeg)} bytes, camera '
            f'{job.camera.name}, {now - job.started:.1f}s after the request'
            + (f', joints pan {joints["pan"]:.3f} tilt {joints["tilt"]:.3f}' if joints else '')
        )
        self.job = None

    def _finish_failed(self, reason):
        job = self.job
        self.job = None
        self.get_logger().error(f'Photo {job.request.request_id} failed: {reason}')
        self._publish(rules.failed(job.request.request_id, job.camera.name, reason))

    def _publish(self, text):
        self.result_publisher.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = PictureTaker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
