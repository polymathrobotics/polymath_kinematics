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
from typing import Optional, Union

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
    """Trajectory for bicycle model.

    ``steering_angle`` is the steady-state / target value used for legend labels and
    analysis plots. For ramped trajectories, ``steering_angle_series`` holds the
    per-sample steering angle (same length as ``time``); the footprint visualizer
    uses it to draw front-wheel indicators at the angle reported for each sample.
    """

    drive_velocity: float = 0.0
    steering_angle: float = 0.0
    turning_radius: float = 0.0
    steering_angle_series: Optional[np.ndarray] = None


@dataclass
class ArticulatedTrajectory(Trajectory):
    """Trajectory for articulated vehicle model.

    ``articulation_angle`` is the steady-state / target value used for legend labels
    and analysis plots. For ramped trajectories, ``articulation_angle_series`` holds
    the per-sample articulation angle (same length as ``time``); the footprint
    visualizer uses it to fold the rear segment progressively across the trajectory.
    """

    drive_velocity: float = 0.0
    articulation_angle: float = 0.0
    turning_radius: float = 0.0
    articulation_angle_series: Optional[np.ndarray] = None


# Type alias for any trajectory type
AnyTrajectory = Union[DifferentialTrajectory, BicycleTrajectory, ArticulatedTrajectory]
