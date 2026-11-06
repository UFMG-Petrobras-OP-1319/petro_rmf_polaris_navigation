"""
Rules of the picture_taker node, kept free of ROS so they can be tested alone.

The fleet asks a robot for a photo with a JSON request and gets a JSON result back,
both as std_msgs/String, so the fleet does not depend on any robot's message types:

  request  {"request_id": "...", "camera": "inspection",
            "expected_joints": {"pan": 0.24, "tilt": 0.44},   # optional
            "timeout_sec": 30.0}                              # optional
  result   {"request_id": "...", "state": "accepted" | "succeeded" | "failed", ...}

The robot only takes the photo when it is ready: the base has stopped moving and, for a
camera on a pan-tilt, the joints sit within tolerance of where the fleet aimed them.
"""

import base64
from dataclasses import dataclass, field
import json
import math
from typing import Optional

import cv2
import numpy as np

ACCEPTED = 'accepted'
SUCCEEDED = 'succeeded'
FAILED = 'failed'

FIXED = 'fixed'
PAN_TILT = 'pan_tilt'
CAMERA_TYPES = (FIXED, PAN_TILT)


@dataclass(frozen=True)
class PictureRequest:
    """One photo the fleet asked for."""

    request_id: str
    camera: str
    expected_joints: Optional[dict] = None
    timeout_sec: Optional[float] = None


@dataclass(frozen=True)
class CameraConfig:
    """How one logical camera (the name the fleet uses) is served on this robot."""

    name: str
    kind: str
    service: str
    joint_state_topic: str = ''
    pan_joint: str = 'pan_joint'
    tilt_joint: str = 'tilt_joint'
    tolerance_rad: float = 0.05
    settle_sec: float = 1.0
    feedback_max_age_sec: float = 1.0


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a number')
    if not math.isfinite(value):
        raise ValueError(f'{name} must be finite')
    return float(value)


def parse_request(text: str) -> PictureRequest:
    """Parse and validate one request; ValueError says which field is wrong."""
    try:
        data = json.loads(text)
    except json.JSONDecodeError as error:
        raise ValueError(f'request is not JSON: {error}') from error
    if not isinstance(data, dict):
        raise ValueError('request must be a JSON object')

    request_id = data.get('request_id')
    if not isinstance(request_id, str) or not request_id.strip():
        raise ValueError('request_id must be a non-empty string')
    camera = data.get('camera')
    if not isinstance(camera, str) or not camera.strip():
        raise ValueError('camera must be a non-empty string')

    expected = data.get('expected_joints')
    if expected is not None:
        if not isinstance(expected, dict) or set(expected) != {'pan', 'tilt'}:
            raise ValueError('expected_joints must be {"pan": <rad>, "tilt": <rad>}')
        expected = {
            axis: _finite_number(expected[axis], f'expected_joints.{axis}')
            for axis in ('pan', 'tilt')
        }

    timeout = data.get('timeout_sec')
    if timeout is not None:
        timeout = _finite_number(timeout, 'timeout_sec')
        if timeout <= 0.0:
            raise ValueError('timeout_sec must be positive')

    return PictureRequest(request_id.strip(), camera.strip(), expected, timeout)


def request_id_of(text: str) -> Optional[str]:
    """Best-effort request_id of a request that failed to parse, to answer it."""
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return None
    value = data.get('request_id') if isinstance(data, dict) else None
    return value if isinstance(value, str) and value.strip() else None


def parse_camera(name: str, values: dict) -> CameraConfig:
    """Build one camera's config from its parameters; ValueError when unusable."""
    camera_type = values.get('type', '')
    if camera_type not in CAMERA_TYPES:
        raise ValueError(
            f'camera {name!r}: type must be one of {CAMERA_TYPES}, got {camera_type!r}')
    service = values.get('service', '')
    if not service:
        raise ValueError(f'camera {name!r}: service is required')
    config = CameraConfig(
        name=name,
        kind=camera_type,
        service=service,
        joint_state_topic=values.get('joint_state_topic', ''),
        pan_joint=values.get('pan_joint', 'pan_joint'),
        tilt_joint=values.get('tilt_joint', 'tilt_joint'),
        tolerance_rad=float(values.get('tolerance_rad', 0.05)),
        settle_sec=float(values.get('settle_sec', 1.0)),
        feedback_max_age_sec=float(values.get('feedback_max_age_sec', 1.0)),
    )
    if config.kind == PAN_TILT and not config.joint_state_topic:
        raise ValueError(f'camera {name!r}: a pan_tilt camera needs joint_state_topic')
    return config


def wrap_pi(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    wrapped = math.remainder(angle, 2.0 * math.pi)
    return wrapped + 2.0 * math.pi if wrapped <= -math.pi else wrapped


def gimbal_errors(expected: dict, measured: dict) -> tuple:
    """
    Return the (pan, tilt) error between where the gimbal is and where it was aimed.

    Pan is compared modulo 2*pi, because a continuous pan joint (or an ISAPI azimuth
    read back as 0~360) can report the same direction as another turn; tilt is not.
    """
    pan = abs(wrap_pi(measured['pan'] - expected['pan']))
    tilt = abs(measured['tilt'] - expected['tilt'])
    return pan, tilt


def joint_positions(names, positions, pan_joint: str, tilt_joint: str) -> Optional[dict]:
    """Pick the pan and tilt joints out of a JointState, or None when either is missing."""
    lookup = dict(zip(names, positions))
    if pan_joint not in lookup or tilt_joint not in lookup:
        return None
    return {'pan': float(lookup[pan_joint]), 'tilt': float(lookup[tilt_joint])}


@dataclass
class StableWindow:
    """
    Track how long a condition has held without interruption.

    Each update says whether the condition holds now; since() is when the current
    unbroken run started, or None while it does not hold.
    """

    _since: Optional[float] = None

    def update(self, holds: bool, now: float) -> None:
        if not holds:
            self._since = None
        elif self._since is None:
            self._since = now

    def since(self) -> Optional[float]:
        return self._since

    def held_for(self, now: float) -> float:
        return 0.0 if self._since is None else now - self._since


@dataclass
class StillnessTracker:
    """
    Decide whether the base is still from a stream of poses.

    The base is still once it stays within linear_m and angular_rad of a reference pose
    for window_sec; leaving that box restarts the window from the new pose.
    """

    linear_m: float
    angular_rad: float
    window_sec: float
    _reference: Optional[tuple] = field(default=None, repr=False)
    _since: Optional[float] = field(default=None, repr=False)

    def update(self, x: float, y: float, yaw: float, now: float) -> None:
        if self._reference is not None:
            ref_x, ref_y, ref_yaw = self._reference
            moved = math.hypot(x - ref_x, y - ref_y) > self.linear_m
            turned = abs(wrap_pi(yaw - ref_yaw)) > self.angular_rad
            if not moved and not turned:
                return
        self._reference = (x, y, yaw)
        self._since = now

    def reset(self) -> None:
        self._reference = None
        self._since = None

    def still_since(self, now: float) -> Optional[float]:
        """When the current still period started, once it has lasted window_sec."""
        if self._since is None or now - self._since < self.window_sec:
            return None
        return self._since


def encode_photo(data: bytes, max_width: int, jpeg_quality: int) -> tuple:
    """
    Return (jpeg_bytes, width, height) of a photo file's contents.

    max_width <= 0 keeps the camera's full resolution, and a JPEG is then sent exactly
    as the camera wrote it; otherwise a wider photo is scaled down to max_width.
    """
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('the camera wrote a file that is not an image')
    height, width = image.shape[:2]
    is_jpeg = data[:2] == b'\xff\xd8'
    if (max_width <= 0 or width <= max_width) and is_jpeg:
        return data, width, height
    if max_width > 0 and width > max_width:
        scale = max_width / width
        image = cv2.resize(
            image, (max_width, max(1, round(height * scale))), interpolation=cv2.INTER_AREA
        )
        height, width = image.shape[:2]
    ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)])
    if not ok:
        raise ValueError('could not encode the photo as JPEG')
    return jpeg.tobytes(), width, height


def accepted(request_id: str, camera: str) -> str:
    return json.dumps({'request_id': request_id, 'state': ACCEPTED, 'camera': camera})


def failed(request_id: str, camera: Optional[str], reason: str) -> str:
    return json.dumps(
        {'request_id': request_id, 'state': FAILED, 'camera': camera, 'reason': reason}
    )


def succeeded(
    request_id: str,
    camera: str,
    jpeg: bytes,
    width: int,
    height: int,
    stamp_sec: float,
    joints: Optional[dict],
    pose: Optional[dict],
) -> str:
    return json.dumps({
        'request_id': request_id,
        'state': SUCCEEDED,
        'camera': camera,
        'stamp': stamp_sec,
        'joints': joints,
        'pose': pose,
        'image': {
            'format': 'jpeg',
            'width': width,
            'height': height,
            'data_base64': base64.b64encode(jpeg).decode('ascii'),
        },
    })
