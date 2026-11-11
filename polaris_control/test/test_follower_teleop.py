"""Pure FSM checks for the joystick STOPPED/teleop ownership fixes."""

from types import SimpleNamespace

import FollowerControlState as states
from follower_control import VectorFollowerNode


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass


class FakeFollower:
    _do_stop_robot = VectorFollowerNode._do_stop_robot
    control_loop = VectorFollowerNode.control_loop

    def __init__(self):
        self.state = states.FollowerControlState.STOPPED
        self.current_vector = object()
        self.orient_target = object()
        self._requested_control_state = states.FollowerControlState.CONTROL_POSITION
        self._last_stop_robot_at = 10.0
        self._teleop_hold_sec = 1.0
        self._stop_robot_flag = False
        self.control_state_applied_count = 0
        self.control_state_rx_count = 0
        self.pose_topic_type = 'unused'
        self.theta = 0.0
        self.cmd_vel_publisher = Publisher()
        self.clear_planner_client = SimpleNamespace(service_is_ready=lambda: True)
        self.get_logger = lambda: Logger()
        self._run_control_position = lambda: setattr(self, 'control_position_ran', True)


def test_stop_while_already_stopped_keeps_explicit_fleet_request(monkeypatch):
    import follower_control

    monkeypatch.setattr(follower_control.time, 'monotonic', lambda: 10.5)
    node = FakeFollower()
    node._do_stop_robot()
    assert node.state == states.FollowerControlState.STOPPED
    assert node._requested_control_state == states.FollowerControlState.CONTROL_POSITION
    assert node.current_vector is None
    assert node.cmd_vel_publisher.messages == []


def test_fleet_resume_waits_until_teleop_keepalives_end(monkeypatch):
    import follower_control

    node = FakeFollower()
    node.current_vector = None
    node._requested_control_state = states.FollowerControlState.CONTROL_POSITION
    monkeypatch.setattr(follower_control.time, 'monotonic', lambda: 10.5)
    node.control_loop()
    assert node.state == states.FollowerControlState.STOPPED
    assert node._requested_control_state == states.FollowerControlState.CONTROL_POSITION
    assert not getattr(node, 'control_position_ran', False)
    assert node.cmd_vel_publisher.messages == []

    monkeypatch.setattr(follower_control.time, 'monotonic', lambda: 11.1)
    node.control_loop()
    assert node.state == states.FollowerControlState.CONTROL_POSITION
    assert node._requested_control_state is None
    assert node.control_position_ran
