"""Opt-in localization health check for the follower's velocity output."""
import math


def localization_ready(status, status_age, pose_receive_age, pose_stamp_age,
                       status_timeout, pose_timeout):
    """Reject missing, stale, future-dated or non-localized inputs."""
    ages = (status_age, pose_receive_age, pose_stamp_age)
    if status != 'LOCALIZED' or not all(math.isfinite(age) for age in ages):
        return False
    return (0 <= status_age <= status_timeout and
            0 <= pose_receive_age <= pose_timeout and
            -0.1 <= pose_stamp_age <= pose_timeout)
