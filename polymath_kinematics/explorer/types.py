# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Trajectory data types for kinematic exploration."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Union

import numpy as np


@dataclass
class Trajectory:
    """Base trajectory data with common fields."""

    time: np.ndarray
    x: np.ndarray
    y: np.ndarray
    theta: np.ndarray
    linear_velocity: float
    angular_velocity: float

    def __len__(self) -> int:
        return len(self.time)


@dataclass
class DifferentialTrajectory(Trajectory):
    """Trajectory for differential drive model."""

    left_wheel: float = 0.0
    right_wheel: float = 0.0
    base_wheel_velocity: float = 0.0


@dataclass
class BicycleTrajectory(Trajectory):
    """Trajectory for bicycle model."""

    drive_velocity: float = 0.0
    steering_angle: float = 0.0
    turning_radius: float = 0.0


@dataclass
class ArticulatedTrajectory(Trajectory):
    """Trajectory for articulated vehicle model."""

    drive_velocity: float = 0.0
    articulation_angle: float = 0.0
    turning_radius: float = 0.0


# Type alias for any trajectory type
AnyTrajectory = Union[DifferentialTrajectory, BicycleTrajectory, ArticulatedTrajectory]
