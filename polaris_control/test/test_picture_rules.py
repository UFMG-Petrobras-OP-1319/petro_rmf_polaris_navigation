"""Rules of the picture_taker node (no ROS needed)."""

import base64
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'src'))

import picture_rules as rules  # noqa: E402,I100


def _request(**fields):
    body = {'request_id': 'scout_1-7-abc', 'camera': 'inspection'}
    body.update(fields)
    return json.dumps(body)


class TestParseRequest:

    def test_minimal_request(self):
        request = rules.parse_request(_request())
        assert request == rules.PictureRequest('scout_1-7-abc', 'inspection', None, None)

    def test_full_request(self):
        request = rules.parse_request(
            _request(expected_joints={'pan': 0.2, 'tilt': -0.1}, timeout_sec=30))
        assert request.expected_joints == {'pan': 0.2, 'tilt': -0.1}
        assert request.timeout_sec == 30.0

    @pytest.mark.parametrize(('text', 'field'), [
        ('not json', 'JSON'),
        ('[1, 2]', 'object'),
        (_request(request_id=''), 'request_id'),
        (_request(camera=3), 'camera'),
        (_request(expected_joints={'pan': 0.1}), 'expected_joints'),
        (_request(expected_joints={'pan': 'a', 'tilt': 0.1}), 'expected_joints.pan'),
        (_request(expected_joints={'pan': True, 'tilt': 0.1}), 'expected_joints.pan'),
        (_request(timeout_sec=0), 'timeout_sec'),
    ])
    def test_invalid_requests_name_the_field(self, text, field):
        with pytest.raises(ValueError, match=field):
            rules.parse_request(text)

    def test_request_id_of_a_broken_request(self):
        assert rules.request_id_of(_request(camera='')) == 'scout_1-7-abc'
        assert rules.request_id_of('not json') is None


class TestCameraConfig:

    def test_fixed_camera(self):
        camera = rules.parse_camera('default', {'type': 'fixed', 'service': 'snap/take_photo'})
        assert camera.kind == rules.FIXED

    def test_pan_tilt_needs_feedback(self):
        with pytest.raises(ValueError, match='joint_state_topic'):
            rules.parse_camera('inspection', {'type': 'pan_tilt', 'service': 'x/take_photo'})

    @pytest.mark.parametrize('values', [
        {'type': 'zoom', 'service': 'x'},
        {'type': 'fixed', 'service': ''},
    ])
    def test_unusable_camera(self, values):
        with pytest.raises(ValueError):
            rules.parse_camera('default', values)


class TestGimbal:

    def test_errors(self):
        assert rules.gimbal_errors({'pan': 0.5, 'tilt': 0.2}, {'pan': 0.45, 'tilt': 0.3}) == (
            pytest.approx(0.05), pytest.approx(0.1))

    def test_pan_is_compared_modulo_two_pi(self):
        pan, _ = rules.gimbal_errors({'pan': math.pi, 'tilt': 0.0}, {'pan': -3.13, 'tilt': 0.0})
        assert pan == pytest.approx(math.pi - 3.13)
        pan, _ = rules.gimbal_errors({'pan': 0.1, 'tilt': 0.0}, {'pan': 0.1 + 2 * math.pi,
                                                                 'tilt': 0.0})
        assert pan == pytest.approx(0.0, abs=1e-9)

    def test_tilt_is_not_wrapped(self):
        _, tilt = rules.gimbal_errors({'pan': 0.0, 'tilt': -3.0}, {'pan': 0.0, 'tilt': 3.0})
        assert tilt == pytest.approx(6.0)

    def test_joint_positions(self):
        names = ['left_wheel', 'pan_joint', 'tilt_joint']
        assert rules.joint_positions(names, [9.0, 0.1, 0.2], 'pan_joint', 'tilt_joint') == {
            'pan': 0.1, 'tilt': 0.2}
        assert rules.joint_positions(names[:2], [9.0, 0.1], 'pan_joint', 'tilt_joint') is None


class TestStableWindow:

    def test_window_restarts_when_broken(self):
        window = rules.StableWindow()
        window.update(True, 1.0)
        window.update(True, 2.0)
        assert window.since() == 1.0
        assert window.held_for(2.5) == 1.5
        window.update(False, 3.0)
        assert window.since() is None
        window.update(True, 4.0)
        assert window.since() == 4.0


class TestStillness:

    def test_still_after_the_window(self):
        tracker = rules.StillnessTracker(linear_m=0.02, angular_rad=0.02, window_sec=1.0)
        tracker.update(1.0, 2.0, 0.5, now=10.0)
        tracker.update(1.005, 2.0, 0.505, now=10.5)
        assert tracker.still_since(10.5) is None
        assert tracker.still_since(11.0) == 10.0

    def test_moving_restarts_the_window(self):
        tracker = rules.StillnessTracker(linear_m=0.02, angular_rad=0.02, window_sec=1.0)
        tracker.update(1.0, 2.0, 0.0, now=10.0)
        tracker.update(1.1, 2.0, 0.0, now=10.8)
        assert tracker.still_since(11.5) is None
        assert tracker.still_since(11.8) == 10.8

    def test_turning_restarts_the_window(self):
        tracker = rules.StillnessTracker(linear_m=0.02, angular_rad=0.02, window_sec=1.0)
        tracker.update(0.0, 0.0, math.pi - 0.001, now=0.0)
        tracker.update(0.0, 0.0, -math.pi + 0.001, now=0.5)  # same heading, wrapped
        assert tracker.still_since(1.0) == 0.0
        tracker.update(0.0, 0.0, 1.0, now=1.2)
        assert tracker.still_since(1.5) is None


def _jpeg(width, height):
    ok, data = cv2.imencode('.jpg', np.full((height, width, 3), 120, dtype=np.uint8))
    assert ok
    return data.tobytes()


class TestEncodePhoto:

    def test_full_resolution_jpeg_is_untouched(self):
        data = _jpeg(64, 48)
        assert rules.encode_photo(data, 0, 90) == (data, 64, 48)

    def test_wide_photo_is_scaled_down(self):
        jpeg, width, height = rules.encode_photo(_jpeg(200, 100), 50, 90)
        assert (width, height) == (50, 25)
        assert cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR).shape == (25, 50, 3)

    def test_png_becomes_jpeg(self):
        ok, png = cv2.imencode('.png', np.zeros((4, 4, 3), dtype=np.uint8))
        jpeg, _, _ = rules.encode_photo(png.tobytes(), 0, 90)
        assert jpeg[:2] == b'\xff\xd8'

    def test_not_an_image(self):
        with pytest.raises(ValueError):
            rules.encode_photo(b'hello', 0, 90)


class TestResults:

    def test_succeeded_carries_the_photo(self):
        result = json.loads(rules.succeeded(
            'id', 'inspection', b'\xff\xd8abc', 4, 3, 12.5,
            {'pan': 0.1, 'tilt': 0.2}, {'x': 1.0, 'y': 2.0, 'yaw': 0.3}))
        assert result['state'] == rules.SUCCEEDED
        assert base64.b64decode(result['image']['data_base64']) == b'\xff\xd8abc'
        assert result['joints'] == {'pan': 0.1, 'tilt': 0.2}

    def test_failed_and_accepted(self):
        assert json.loads(rules.failed('id', 'inspection', 'why')) == {
            'request_id': 'id', 'state': 'failed', 'camera': 'inspection', 'reason': 'why'}
        assert json.loads(rules.accepted('id', 'default'))['state'] == 'accepted'
