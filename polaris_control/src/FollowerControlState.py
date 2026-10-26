# Copyright 2026 UFMG-Petrobras OP-1319
#
# This file is part of petro_rmf_utils.
#
# petro_rmf_utils is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

"""Shared follower finite-state-machine states."""

from enum import Enum


class FollowerControlState(Enum):
    """States used by the Polaris follower controller."""

    STOPPED = "STOPPED"
    CONTROL_POSITION = "CONTROL_POSITION"
    ALIGN_YAW = "ALIGN_YAW"
