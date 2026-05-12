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
"""Export functions for trajectory data."""

from __future__ import annotations

import numpy as np
import pandas as pd

from .types import AnyTrajectory, ArticulatedTrajectory, BicycleTrajectory, DifferentialTrajectory


def trajectories_to_dataframe(
    trajectories: list[AnyTrajectory],
    model_type: str,
) -> pd.DataFrame:
    """Convert trajectories to a pandas DataFrame for export.

    Uses vectorized construction for better performance.

    Args:
        trajectories: List of trajectory dataclasses
        model_type: One of "Differential Drive", "Bicycle", or "Articulated"

    Returns:
        DataFrame with trajectory data, columns depend on model type
    """
    if not trajectories:
        return pd.DataFrame()

    # Calculate total size for preallocation
    num_total = sum(len(trajectory) for trajectory in trajectories)

    # Preallocate arrays for common columns
    trajectory_ids = np.empty(num_total, dtype=int)
    time_array = np.empty(num_total)
    x_array = np.empty(num_total)
    y_array = np.empty(num_total)
    theta_array = np.empty(num_total)
    linear_velocity_array = np.empty(num_total)
    angular_velocity_array = np.empty(num_total)

    # Model-specific arrays
    if model_type == 'Differential Drive':
        left_wheel_array = np.empty(num_total)
        right_wheel_array = np.empty(num_total)
        base_wheel_velocity_array = np.empty(num_total)
    elif model_type == 'Bicycle':
        drive_velocity_array = np.empty(num_total)
        steering_angle_array = np.empty(num_total)
        turning_radius_array = np.empty(num_total)
    elif model_type == 'Articulated':
        drive_velocity_array = np.empty(num_total)
        articulation_angle_array = np.empty(num_total)
        turning_radius_array = np.empty(num_total)

    # Fill arrays
    current_index = 0
    for trajectory_index, trajectory in enumerate(trajectories):
        num_points = len(trajectory)
        end_index = current_index + num_points

        trajectory_ids[current_index:end_index] = trajectory_index
        time_array[current_index:end_index] = trajectory.time
        x_array[current_index:end_index] = trajectory.x
        y_array[current_index:end_index] = trajectory.y
        theta_array[current_index:end_index] = trajectory.theta
        linear_velocity_array[current_index:end_index] = trajectory.linear_velocity
        angular_velocity_array[current_index:end_index] = trajectory.angular_velocity

        if model_type == 'Differential Drive':
            trajectory_differential: DifferentialTrajectory = trajectory  # type: ignore[assignment]
            left_wheel_array[current_index:end_index] = trajectory_differential.left_wheel
            right_wheel_array[current_index:end_index] = trajectory_differential.right_wheel
            base_wheel_velocity_array[current_index:end_index] = trajectory_differential.base_wheel_velocity
        elif model_type == 'Bicycle':
            trajectory_bicycle: BicycleTrajectory = trajectory  # type: ignore[assignment]
            drive_velocity_array[current_index:end_index] = trajectory_bicycle.drive_velocity
            steering_angle_array[current_index:end_index] = trajectory_bicycle.steering_angle
            turning_radius_array[current_index:end_index] = trajectory_bicycle.turning_radius
        elif model_type == 'Articulated':
            trajectory_articulated: ArticulatedTrajectory = trajectory  # type: ignore[assignment]
            drive_velocity_array[current_index:end_index] = trajectory_articulated.drive_velocity
            articulation_angle_array[current_index:end_index] = trajectory_articulated.articulation_angle
            turning_radius_array[current_index:end_index] = trajectory_articulated.turning_radius

        current_index = end_index

    # Build DataFrame from arrays (single allocation)
    data: dict[str, np.ndarray] = {
        'trajectory_id': trajectory_ids,
        'time': time_array,
        'x': x_array,
        'y': y_array,
        'theta': theta_array,
        'linear_velocity': linear_velocity_array,
        'angular_velocity': angular_velocity_array,
    }

    if model_type == 'Differential Drive':
        data['left_wheel_velocity'] = left_wheel_array
        data['right_wheel_velocity'] = right_wheel_array
        data['base_wheel_velocity'] = base_wheel_velocity_array
    elif model_type == 'Bicycle':
        data['drive_velocity'] = drive_velocity_array
        data['steering_angle'] = steering_angle_array
        data['turning_radius'] = turning_radius_array
    elif model_type == 'Articulated':
        data['drive_velocity'] = drive_velocity_array
        data['articulation_angle'] = articulation_angle_array
        data['turning_radius'] = turning_radius_array

    return pd.DataFrame(data)
