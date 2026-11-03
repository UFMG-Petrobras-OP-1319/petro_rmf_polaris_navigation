"""Unit tests for the pure follower state-machine rules."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parents[1] / 'src'))

from follower_fsm import apply_control_request  # noqa: E402
from follower_fsm import is_external_command_fresh  # noqa: E402
from follower_fsm import parse_control_request  # noqa: E402
from follower_fsm import vector_starts_control  # noqa: E402
from FollowerControlState import FollowerControlState  # noqa: E402
import pytest  # noqa: E402

STOPPED = FollowerControlState.STOPPED
CONTROL_POSITION = FollowerControlState.CONTROL_POSITION
ALIGN_YAW = FollowerControlState.ALIGN_YAW
EXTERNAL = FollowerControlState.EXTERNAL


@pytest.mark.parametrize('text, expected', [
    ('CONTROL_POSITION', CONTROL_POSITION),
    (' ALIGN_YAW\n', ALIGN_YAW),
    ('EXTERNAL', EXTERNAL),
    ('STOPPED', None),
    ('PAN_TILT', None),
    ('', None),
])
def test_parse_control_request(text, expected):
    assert parse_control_request(text) == expected


@pytest.mark.parametrize('current', [STOPPED, CONTROL_POSITION, ALIGN_YAW, EXTERNAL])
def test_external_is_accepted_from_every_state(current):
    assert apply_control_request(current, EXTERNAL) == EXTERNAL


@pytest.mark.parametrize('requested', [CONTROL_POSITION, ALIGN_YAW])
def test_requests_stay_pending_while_stopped(requested):
    assert apply_control_request(STOPPED, requested) is None


@pytest.mark.parametrize('requested', [CONTROL_POSITION, ALIGN_YAW])
def test_explicit_request_leaves_external(requested):
    assert apply_control_request(EXTERNAL, requested) == requested


def test_vectors_only_wake_a_stopped_follower():
    assert vector_starts_control(STOPPED)
    assert not vector_starts_control(EXTERNAL)
    assert not vector_starts_control(ALIGN_YAW)
    assert not vector_starts_control(CONTROL_POSITION)


@pytest.mark.parametrize('received, now, fresh', [
    (None, 10.0, False),
    (10.0, 10.0, True),
    (10.0, 10.25, True),
    (10.0, 10.26, False),
    (10.5, 10.0, False),
])
def test_external_command_watchdog(received, now, fresh):
    assert is_external_command_fresh(received, now, 0.25) is fresh
