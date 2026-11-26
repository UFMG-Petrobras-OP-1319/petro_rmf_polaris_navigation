"""Offline return lifecycle checks: no ROS nodes, publishers, services or hardware."""

from collections import deque
from concurrent.futures import Future
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'src'))
import picture_rules as rules
from picture_taker import CAPTURING, Job, PictureTaker, RETURNING


def camera(**overrides):
    values = dict(
        type='pan_tilt', service='take_photo', joint_state_topic='joints',
        return_to_forward=True, forward_pose_calibrated=True,
        forward_pan_rad=0.2, forward_tilt_rad=-0.1,
        pan_cmd_topic='pan', tilt_cmd_topic='tilt',
    )
    values.update(overrides)
    return rules.parse_camera('inspection', values)


@pytest.mark.parametrize('overrides', [
    {'forward_pose_calibrated': False}, {'pan_cmd_topic': ''},
    {'tilt_cmd_topic': 'pan'}, {'return_timeout_sec': 0},
    {'forward_pan_rad': math.nan}, {'forward_tilt_rad': math.inf},
    {'return_tolerance_rad': -0.1}, {'settle_sec': 0},
    {'feedback_max_age_sec': 0}, {'return_to_forward': 'true'},
])
def test_return_rejects_unsafe_configuration(overrides):
    with pytest.raises(ValueError):
        camera(**overrides)


def test_no_motion_without_fresh_feedback_or_both_subscribers():
    returning = rules.ForwardReturn(camera(), 10.0, 20.0)
    away = {'pan': 1.0, 'tilt': 0.5}
    assert returning.update(10.1, away, 9.9, True) == 'waiting'
    assert returning.update(10.2, away, 10.2, False) == 'waiting'
    assert returning.commanded_at is None
    assert returning.update(10.3, away, 10.3, True) == 'command'
    # An unchanged/off-target camera must not be fought with repeated targets.
    assert returning.update(11.0, away, 11.0, True) == 'waiting'
    assert returning.update(20.0, away, 20.0, True) == 'failed'


def test_settling_requires_new_samples_and_both_axes_at_forward():
    returning = rules.ForwardReturn(camera(), 10.0, 20.0)
    home = {'pan': 0.2 + 2 * math.pi, 'tilt': -0.1}
    assert returning.update(10.1, home, 10.1, True) == 'waiting'
    assert returning.update(11.1, home, 10.1, True) == 'waiting'
    assert returning.update(11.2, {'pan': 0.2, 'tilt': 0.0}, 11.2, True) == 'command'
    assert returning.update(11.3, home, 11.2, True) == 'waiting'
    assert returning.update(11.4, home, 11.4, True) == 'waiting'
    assert returning.update(12.5, home, 12.5, True) == 'done'


def test_stale_or_nonfinite_feedback_breaks_the_settling_window():
    returning = rules.ForwardReturn(camera(), 10.0, 20.0)
    home = {'pan': 0.2, 'tilt': -0.1}
    assert returning.update(10.1, home, 10.1, True) == 'waiting'
    assert returning.update(11.2, home, 10.1, True) == 'waiting'
    assert returning.update(11.3, home, 11.3, True) == 'waiting'
    assert returning.update(11.8, {'pan': math.nan, 'tilt': -0.1}, 11.8, True) == 'waiting'
    assert returning.update(12.0, home, 12.0, True) == 'waiting'
    assert returning.update(12.5, home, 12.5, True) == 'waiting'
    assert returning.update(13.1, home, 13.1, True) == 'done'
    assert rules.joint_positions(['pan', 'tilt'], [math.nan, 0], 'pan', 'tilt') is None


class Publisher:
    def __init__(self):
        self.messages = []

    def get_subscription_count(self):
        return 1

    def publish(self, msg):
        self.messages.append(msg.data if hasattr(msg, 'data') else (msg.x, msg.y, msg.z))


class Harness:
    # Execute the actual node's lifecycle methods without constructing a ROS node.
    _check_capture = PictureTaker._check_capture
    _photo_path = PictureTaker._photo_path
    _finish_failed = PictureTaker._finish_failed
    _complete_or_return = PictureTaker._complete_or_return
    _check_return = PictureTaker._check_return
    _finish_result = PictureTaker._finish_result
    _on_request = PictureTaker._on_request
    _on_return_position = PictureTaker._on_return_position
    _measured_return_position = PictureTaker._measured_return_position
    _readiness = PictureTaker._readiness
    _retry_capture = PictureTaker._retry_capture

    def __init__(self, tmp_path, config):
        self.time = 10.0
        self.job = Job(rules.PictureRequest('shot-1', config.name), config, 0.0, 15.0)
        self.job.result_deadline = 30.0
        self.photo_dir = str(tmp_path)
        self.keep_photos = False
        self.max_width = 0
        self.jpeg_quality = 90
        self.pose = {'x': 1.0, 'y': 2.0, 'yaw': 0.5}
        self.joints = {'pan': 1.0, 'tilt': 0.5}
        self.job.capture_pose = dict(self.pose)
        self.job.capture_joints = dict(self.joints)
        self.age = 0.0
        publishers = (Publisher(),) if config.return_command_topic else (Publisher(), Publisher())
        self.return_publishers = {config.name: publishers}
        self.return_positions = {}
        self.results = []
        self.seen_requests = deque(maxlen=64)
        self.capture_delivery = SimpleNamespace(
            has_capture=lambda _id: False, enqueue=lambda *_: None)
        self.get_parameter = lambda _name: SimpleNamespace(value=False)
        self.tf_inertial_link = 'scout_1/map'
        self.tf_robot_pose = 'scout_1/base_link'
        self.camera_errors = {}
        self.cameras = {config.name: config}
        self.ready_timeout = 15.0
        self.still_window = 1.0
        self.service_timeout = 5.0

    def _now(self):
        return self.time

    def _measured_joints(self, _camera):
        return self.joints, self.age

    def _publish(self, text):
        self.results.append(json.loads(text))

    def get_logger(self):
        return SimpleNamespace(info=lambda *_: None, error=lambda *_: None)


def captured_job(tmp_path, config, delivery=None):
    import cv2
    import numpy as np
    node = Harness(tmp_path, config)
    if delivery is not None:
        node.capture_delivery = delivery
    ok, image = cv2.imencode('.jpg', np.zeros((8, 12, 3), dtype=np.uint8))
    assert ok
    Path(node._photo_path()).write_bytes(image.tobytes())
    node.job.phase = CAPTURING
    node.job.ready_since = 8.0
    node.job.future = Future()
    node.job.future.set_result(SimpleNamespace(
        success=True, message='', stamp=SimpleNamespace(sec=9, nanosec=0)))
    node._check_capture(10.0)
    return node


def test_picture_success_waits_for_return_and_keeps_capture_metadata(tmp_path):
    node = captured_job(tmp_path, camera())
    assert node.job.phase == RETURNING
    assert node.results == []
    photo = Path(node._photo_path())
    assert photo.exists()
    node._check_return(10.1)
    pubs = node.return_publishers['inspection']
    assert pubs[0].messages == [0.2]
    assert pubs[1].messages == [-0.1]
    node.joints = {'pan': 0.2, 'tilt': -0.1}
    node._check_return(10.2)
    node._check_return(11.3)
    assert node.job is None
    assert node.results[0]['state'] == 'succeeded'
    # Image metadata must describe the shot, not the later home orientation.
    assert node.results[0]['joints'] == {'pan': 1.0, 'tilt': 0.5}
    assert node.results[0]['stamp'] == 9.0
    assert node.results[0]['resting_joints'] == {'pan': 0.2, 'tilt': -0.1}
    assert not photo.exists()


def test_failed_return_keeps_the_photo_and_reports_failure(tmp_path):
    node = captured_job(tmp_path, camera())
    photo = Path(node._photo_path())
    node._check_return(10.1)
    node._check_return(20.0)
    assert node.job is None
    assert node.results[0]['state'] == 'failed'
    assert 'return to forward failed' in node.results[0]['reason']
    assert str(photo) in node.results[0]['reason']
    assert 'resting_joints' not in node.results[0]
    assert photo.exists()


def test_capture_failure_remains_failed_after_successful_return(tmp_path):
    node = Harness(tmp_path, camera())
    node._finish_failed('camera disconnected')
    assert node.results == []
    node._check_return(10.1)
    node.joints = {'pan': 0.2, 'tilt': -0.1}
    node._check_return(10.2)
    node._check_return(11.3)
    assert node.results[0]['state'] == 'failed'
    assert node.results[0]['reason'] == 'camera disconnected'


def test_new_request_cannot_supersede_an_active_return(tmp_path):
    node = captured_job(tmp_path, camera())
    original = node.job
    node._on_request(SimpleNamespace(data=json.dumps(
        {'request_id': 'shot-2', 'camera': 'inspection'})))
    assert node.job is original
    assert node.results[0]['request_id'] == 'shot-2'
    assert node.results[0]['state'] == 'failed'


def test_fixed_camera_never_sends_ptz_commands(tmp_path):
    fixed = rules.parse_camera('fixed', {'type': 'fixed', 'service': 'snapshot'})
    node = captured_job(tmp_path, fixed)
    assert node.job is None
    assert node.results[0]['state'] == 'succeeded'
    assert all(not pub.messages for pub in node.return_publishers['fixed'])


def test_collector_outbox_remains_durable_when_the_return_fails(tmp_path):
    from capture_delivery import CaptureDelivery
    delivery = CaptureDelivery(tmp_path / 'outbox', 'scout_1', start_worker=False)
    try:
        node = captured_job(tmp_path, camera(), delivery)
        assert delivery.has_capture('shot-1')
        assert node.results == []
        node._check_return(20.0)
        assert node.results[0]['state'] == 'failed'
    finally:
        delivery.close()
    reopened = CaptureDelivery(tmp_path / 'outbox', 'scout_1', start_worker=False)
    try:
        assert reopened.has_capture('shot-1')
    finally:
        reopened.close()


def test_request_budget_reserves_time_to_return_before_fleet_timeout(tmp_path):
    node = Harness(tmp_path, camera())
    node.job = None
    node._on_request(SimpleNamespace(data=json.dumps(dict(
        request_id='shot-2', camera='inspection', timeout_sec=30.0))))
    assert node.job.deadline + node.service_timeout + 10.0 == node.job.result_deadline
    assert node.job.result_deadline < node.time + 30.0


def test_atomic_return_restores_zoom_and_waits_for_it_before_success(tmp_path):
    config = camera(return_command_topic='joint_absolute', return_position_topic='joint_position')
    node = captured_job(tmp_path, config)

    def feedback(now, pan, tilt, zoom):
        node.time = now
        node.return_positions[config.name] = (
            {'pan': pan, 'tilt': tilt, 'zoom': zoom}, now)
        node._check_return(now)

    feedback(10.1, 1.0, 0.5, 8.0)
    assert node.return_publishers[config.name][0].messages == [(0.2, -0.1, 1.0)]
    feedback(10.2, 0.2, -0.1, 2.0)
    feedback(11.3, 0.2, -0.1, 2.0)
    assert node.results == []  # pan/tilt alone do not establish the requested view
    feedback(11.5, 0.2, -0.1, 1.0)
    feedback(12.6, 0.2, -0.1, 1.0)
    assert node.results[0]['state'] == 'succeeded'
    assert node.results[0]['resting_zoom'] == 1.0
    assert node.results[0]['resting_joints'] == {'pan': 0.2, 'tilt': -0.1}
    assert len(node.return_publishers[config.name][0].messages) == 1


def test_atomic_return_requires_stamped_feedback(tmp_path):
    config = camera(return_command_topic='joint_absolute', return_position_topic='joint_position')
    node = captured_job(tmp_path, config)
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
                          vector=SimpleNamespace(x=0.2, y=-0.1, z=1.0))
    node._on_return_position(config.name, msg)
    node._check_return(10.1)
    assert node.return_publishers[config.name][0].messages == []


@pytest.mark.parametrize('overrides', [
    {'return_command_topic': 'joint_absolute'},
    {'return_position_topic': 'joint_position'},
    {'forward_zoom': 0}, {'forward_zoom': math.inf},
])
def test_atomic_return_rejects_incomplete_configuration(overrides):
    with pytest.raises(ValueError):
        camera(**overrides)


@pytest.mark.parametrize('pose_time, initially_still', [
    (None, None),       # no robot pose
    (8.0, 7.0),         # stale robot pose, even with an old stillness window
    (10.0, None),       # fresh pose, moving base
    (10.0, 9.0),        # already stationary
])
def test_manager_cached_target_is_reissued_once_after_the_camera_returned_home(
        pose_time, initially_still):
    config = camera(aim_pan_cmd_topic='pan', aim_tilt_cmd_topic='tilt')
    node = Harness(Path('/tmp'), config)
    node.job.request = rules.PictureRequest(
        'shot-2', 'inspection', {'pan': 0.2, 'tilt': -0.1}, 30.0)
    node.job.started = 10.0
    node.joints = {'pan': 0.0, 'tilt': 0.0}
    node.pose_time = pose_time
    node.pose_max_age = 1.0
    node.stillness = SimpleNamespace(still_since=lambda _now: initially_still)
    node.clients_by_camera = {'inspection': SimpleNamespace(service_is_ready=lambda: True)}
    node.aim_publishers = {'inspection': (Publisher(), Publisher())}

    ready, reasons = node._readiness(10.0)
    assert ready is None
    if pose_time is None or pose_time < 9.0 or initially_still is None:
        assert node.aim_publishers['inspection'][0].messages == []
        assert node.aim_publishers['inspection'][1].messages == []
        assert not node.job.aim_commanded
        node.pose_time = 10.0
        node.stillness = SimpleNamespace(still_since=lambda _now: 9.0)
        ready, reasons = node._readiness(10.0)
        assert ready is None
    assert 're-aiming the PTZ once to the manager target' in reasons
    assert node.aim_publishers['inspection'][0].messages == [0.2]
    assert node.aim_publishers['inspection'][1].messages == [-0.1]

    node._readiness(10.1)
    assert node.aim_publishers['inspection'][0].messages == [0.2]
    assert node.aim_publishers['inspection'][1].messages == [-0.1]


def test_frame_readiness_starts_after_full_base_and_gimbal_settling(tmp_path):
    node = Harness(tmp_path, camera(settle_sec=3.0))
    node.still_window = 2.0
    node.pose_max_age = 1.0
    node.stillness = SimpleNamespace(still_since=lambda _now: 7.0)
    node.clients_by_camera = {'inspection': SimpleNamespace(service_is_ready=lambda: True)}
    node.job.request = rules.PictureRequest(
        'shot-1', 'inspection', dict(node.joints), 30.0)
    for now in (10.0, 12.9):
        node.pose_time = now
        ready, reasons = node._readiness(now)
        assert ready is None
        assert 'gimbal settling' in reasons
    node.pose_time = 13.0
    ready, reasons = node._readiness(13.0)
    assert not reasons
    assert ready == 13.0  # start at 10 plus all three settling seconds
    # A later base stillness window also bounds the accepted frame timestamp.
    node.stillness = SimpleNamespace(still_since=lambda _now: 12.0)
    node.pose_time = 14.0
    ready, reasons = node._readiness(14.0)
    assert not reasons
    assert ready == 14.0


def test_cached_frame_from_inside_settling_window_is_retried(tmp_path):
    node = Harness(tmp_path, camera(settle_sec=3.0))
    node.job.phase = CAPTURING
    node.job.ready_since = 13.0
    node.job.deadline = 20.0
    node.job.future = Future()
    node.job.future.set_result(SimpleNamespace(
        success=True, message='', stamp=SimpleNamespace(sec=12, nanosec=800000000)))
    node._check_capture(14.0)
    assert node.job.phase == 'waiting'
    assert node.job.last_camera_error == 'frame older than the ready moment'
    assert node.results == []
    assert not Path(node._photo_path()).exists()
