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
        self.messages.append(msg.data)


class Harness:
    # Execute the actual node's lifecycle methods without constructing a ROS node.
    _check_capture = PictureTaker._check_capture
    _photo_path = PictureTaker._photo_path
    _finish_failed = PictureTaker._finish_failed
    _complete_or_return = PictureTaker._complete_or_return
    _check_return = PictureTaker._check_return
    _finish_result = PictureTaker._finish_result
    _on_request = PictureTaker._on_request

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
        self.age = 0.0
        self.return_publishers = {config.name: (Publisher(), Publisher())}
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
