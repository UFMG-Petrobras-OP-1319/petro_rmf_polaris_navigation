"""Pure transition rules for the Polaris follower state machine.

Kept free of ROS imports so the rules can be unit tested without a ROS graph.
"""

from FollowerControlState import FollowerControlState


REQUESTABLE_STATES = frozenset({
    FollowerControlState.CONTROL_POSITION,
    FollowerControlState.ALIGN_YAW,
    FollowerControlState.EXTERNAL,
})


def parse_control_request(text):
    """Return the requested state for an explicit command, or None if invalid."""
    command = text.strip()
    for state in REQUESTABLE_STATES:
        if state.value == command:
            return state
    return None


def apply_control_request(current, requested):
    """Return the state after applying ``requested``, or None to keep it pending.

    EXTERNAL hands the base to another velocity source (for example the Nav2
    docking server) and is accepted from every state, including STOPPED.
    CONTROL_POSITION and ALIGN_YAW stay pending while the follower is STOPPED,
    which preserves the original stop_robot contract.
    """
    if requested == FollowerControlState.EXTERNAL:
        return FollowerControlState.EXTERNAL
    if current == FollowerControlState.STOPPED:
        return None
    return requested


def vector_starts_control(current):
    """Return True when a new path vector should leave STOPPED.

    Vectors never pull the follower out of EXTERNAL; only an explicit
    CONTROL_POSITION or ALIGN_YAW request ends external control.
    """
    return current == FollowerControlState.STOPPED


def is_external_command_fresh(received_sec, now_sec, timeout_sec):
    """Return True when the last external command may still be forwarded."""
    if received_sec is None:
        return False
    age = now_sec - received_sec
    return 0.0 <= age <= timeout_sec
