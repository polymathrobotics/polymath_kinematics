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
"""Trajectory simulation and lattice generation functions.

Bicycle and articulated lattices are generated via the C++ BicycleProjector /
ArticulatedProjector forward-simulation classes (Euler integration with
steering-rate ramping support). The differential drive lattice keeps a
vectorized Euler integration here because there is no DifferentialDriveProjector
in the C++ library.

All functions are pure (no streamlit dependency) for testability and reuse.
"""

from __future__ import annotations

import numpy as np

from polymath_kinematics import (
    ArticulatedModel,
    ArticulatedProjector,
    BicycleModel,
    BicycleProjector,
    DifferentialDriveModel,
    DifferentialDriveProjector,
    Pose2D,
)

from .types import ArticulatedTrajectory, BicycleTrajectory, DifferentialTrajectory


def simulate_trajectory_euler(
    linear_velocity: float,
    angular_velocity: float,
    duration: float = 5.0,
    time_step: float = 0.02,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Simulate trajectory using vectorized Euler integration.

    For constant angular velocity, theta is analytic: theta(t) = omega * t.
    This allows fully vectorized computation of x and y via cumulative sum.

    Returns:
        Tuple of (time, x, y, theta) arrays
    """
    num_steps = int(duration / time_step)
    time_array = np.linspace(0, duration, num_steps)

    # Theta is analytic for constant angular velocity
    theta = angular_velocity * time_array

    # Compute velocities at each timestep (vectorized)
    velocity_x = linear_velocity * np.cos(theta)
    velocity_y = linear_velocity * np.sin(theta)

    # Integrate via cumulative sum (Euler method, vectorized)
    x = np.concatenate([[0], np.cumsum(velocity_x[:-1]) * time_step])
    y = np.concatenate([[0], np.cumsum(velocity_y[:-1]) * time_step])

    return time_array, x, y, theta


def simulate_trajectory_rk4(
    linear_velocity: float,
    angular_velocity: float,
    duration: float = 5.0,
    time_step: float = 0.02,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Simulate trajectory using 4th-order Runge-Kutta integration.

    For constant angular velocity, we can still vectorize theta, then use
    RK4-style weighted averaging for x and y positions.

    Returns:
        Tuple of (time, x, y, theta) arrays
    """
    num_steps = int(duration / time_step)
    time_array = np.linspace(0, duration, num_steps)

    theta = angular_velocity * time_array
    theta_prev = theta[:-1]
    theta_mid = theta_prev + angular_velocity * time_step / 2
    theta_next = theta_prev + angular_velocity * time_step

    k1_x = linear_velocity * np.cos(theta_prev)
    k1_y = linear_velocity * np.sin(theta_prev)
    k2_x = linear_velocity * np.cos(theta_mid)
    k2_y = linear_velocity * np.sin(theta_mid)
    k4_x = linear_velocity * np.cos(theta_next)
    k4_y = linear_velocity * np.sin(theta_next)

    delta_x = (k1_x + 4 * k2_x + k4_x) / 6 * time_step
    delta_y = (k1_y + 4 * k2_y + k4_y) / 6 * time_step

    x = np.concatenate([[0], np.cumsum(delta_x)])
    y = np.concatenate([[0], np.cumsum(delta_y)])

    return time_array, x, y, theta


def simulate_trajectory(
    linear_velocity: float,
    angular_velocity: float,
    duration: float = 5.0,
    time_step: float = 0.02,
    method: str = 'euler',
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Simulate a constant-cmd_vel trajectory.

    Args:
        linear_velocity: Linear velocity (m/s)
        angular_velocity: Angular velocity (rad/s)
        duration: Simulation duration (s)
        time_step: Time step (s)
        method: Integration method ("euler" or "rk4")
    """
    if method == 'rk4':
        return simulate_trajectory_rk4(linear_velocity, angular_velocity, duration, time_step)
    return simulate_trajectory_euler(linear_velocity, angular_velocity, duration, time_step)


def _states_to_arrays(states):
    """Pull (time, x, y, theta, omega) arrays out of a list of ProjectedState samples."""
    n = len(states)
    time_arr = np.empty(n)
    x_arr = np.empty(n)
    y_arr = np.empty(n)
    theta_arr = np.empty(n)
    omega_arr = np.empty(n)
    for i, s in enumerate(states):
        time_arr[i] = s.time_s
        x_arr[i] = s.pose.x
        y_arr[i] = s.pose.y
        theta_arr[i] = s.pose.theta
        omega_arr[i] = s.angular_velocity_rad_s
    return time_arr, x_arr, y_arr, theta_arr, omega_arr


def generate_lattice_differential(
    wheel_radius: float,
    track_width: float,
    base_wheel_velocities: tuple[float, ...],
    wheel_velocity_diffs: tuple[float, ...],
    duration: float,
    time_step: float = 0.02,
    method: str = 'euler',
) -> list[DifferentialTrajectory]:
    """Generate trajectory lattice by sweeping wheel velocity differences."""
    model = DifferentialDriveModel(wheel_radius, track_width)
    trajectories: list[DifferentialTrajectory] = []

    for base_velocity in base_wheel_velocities:
        for velocity_diff in wheel_velocity_diffs:
            left_wheel = base_velocity - velocity_diff / 2
            right_wheel = base_velocity + velocity_diff / 2

            body_velocity = model.wheel_velocities_to_body_velocity(left_wheel, right_wheel)
            linear_velocity = body_velocity.linear_velocity_m_s
            angular_velocity = body_velocity.angular_velocity_rad_s

            time_array, x, y, theta = simulate_trajectory(
                linear_velocity, angular_velocity, duration, time_step, method
            )

            trajectories.append(
                DifferentialTrajectory(
                    time=time_array,
                    x=x,
                    y=y,
                    theta=theta,
                    linear_velocity=linear_velocity,
                    angular_velocity=angular_velocity,
                    left_wheel=left_wheel,
                    right_wheel=right_wheel,
                    base_wheel_velocity=base_velocity,
                )
            )

    return trajectories


def generate_lattice_bicycle(
    wheelbase: float,
    track_width: float,
    wheel_radius: float,
    drive_velocities: tuple[float, ...],
    steering_angles: tuple[float, ...],
    duration: float,
    time_step: float = 0.02,
    method: str = 'euler',  # accepted for API compatibility; projector is Euler-only
    min_steering_angle_rad: float | None = None,
    max_steering_angle_rad: float | None = None,
    steering_rate_rad_s: float = 0.0,
) -> list[BicycleTrajectory]:
    """Generate trajectory lattice by sweeping steering angles.

    Each lattice cell is generated via BicycleProjector.project() with the
    initial steering angle equal to the target, so no ramp occurs and the
    behavior matches a constant-steering sweep. ``steering_rate_rad_s`` is
    exposed for future use; with a non-zero value and ``min/max`` set, the
    projector will ramp toward the target — but a constant-input lattice
    is the typical use.

    Note: ``method`` is accepted for backward compatibility but ignored. The
    projector uses Euler integration only; RK4 trajectory generation has
    been retired in favor of a single forward-simulation codepath.
    """
    del method  # accepted for API compat; projector is Euler-only.
    angles = list(steering_angles)
    if not angles:
        return []
    min_angle = min_steering_angle_rad if min_steering_angle_rad is not None else min(angles)
    max_angle = max_steering_angle_rad if max_steering_angle_rad is not None else max(angles)

    model = BicycleModel(wheelbase, track_width, wheel_radius)
    projector = BicycleProjector(model, min_angle, max_angle)

    trajectories: list[BicycleTrajectory] = []
    for drive_velocity in drive_velocities:
        for steering_angle in angles:
            states = projector.project(
                horizon_s=duration,
                dt_s=time_step,
                initial_pose=Pose2D(),
                initial_steering_angle_rad=steering_angle,
                target_steering_angle_rad=steering_angle,
                steering_rate_rad_s=steering_rate_rad_s,
                linear_velocity_m_s=drive_velocity,
            )
            time_arr, x_arr, y_arr, theta_arr, _omega = _states_to_arrays(states)

            trajectories.append(
                BicycleTrajectory(
                    time=time_arr,
                    x=x_arr,
                    y=y_arr,
                    theta=theta_arr,
                    linear_velocity=drive_velocity,
                    angular_velocity=states[0].angular_velocity_rad_s,
                    drive_velocity=drive_velocity,
                    steering_angle=steering_angle,
                    turning_radius=states[0].steering_state.turning_radius_m,
                )
            )

    return trajectories


def generate_lattice_articulated(
    articulation_to_front: float,
    articulation_to_rear: float,
    front_track: float,
    rear_track: float,
    front_wheel_radius: float,
    rear_wheel_radius: float,
    drive_velocities: tuple[float, ...],
    articulation_angles: tuple[float, ...],
    duration: float,
    time_step: float = 0.02,
    method: str = 'euler',  # accepted for API compatibility; projector is Euler-only
    min_articulation_angle_rad: float | None = None,
    max_articulation_angle_rad: float | None = None,
    articulation_rate_rad_s: float = 0.0,
) -> list[ArticulatedTrajectory]:
    """Generate trajectory lattice by sweeping articulation angles.

    See ``generate_lattice_bicycle`` for the projector-based rationale and
    the meaning of ``method``.
    """
    del method  # accepted for API compat; projector is Euler-only.
    angles = list(articulation_angles)
    if not angles:
        return []
    min_angle = min_articulation_angle_rad if min_articulation_angle_rad is not None else min(angles)
    max_angle = max_articulation_angle_rad if max_articulation_angle_rad is not None else max(angles)

    model = ArticulatedModel(
        articulation_to_front,
        articulation_to_rear,
        front_track,
        rear_track,
        front_wheel_radius,
        rear_wheel_radius,
    )
    projector = ArticulatedProjector(model, min_angle, max_angle)

    trajectories: list[ArticulatedTrajectory] = []
    for drive_velocity in drive_velocities:
        for articulation_angle in angles:
            states = projector.project(
                horizon_s=duration,
                dt_s=time_step,
                initial_pose=Pose2D(),
                initial_articulation_angle_rad=articulation_angle,
                target_articulation_angle_rad=articulation_angle,
                articulation_rate_rad_s=articulation_rate_rad_s,
                linear_velocity_m_s=drive_velocity,
            )
            time_arr, x_arr, y_arr, theta_arr, _omega = _states_to_arrays(states)

            trajectories.append(
                ArticulatedTrajectory(
                    time=time_arr,
                    x=x_arr,
                    y=y_arr,
                    theta=theta_arr,
                    linear_velocity=drive_velocity,
                    angular_velocity=states[0].angular_velocity_rad_s,
                    drive_velocity=drive_velocity,
                    articulation_angle=articulation_angle,
                    turning_radius=states[0].vehicle_state.front_axle_turning_radius_m,
                )
            )

    return trajectories


# ----------------------------------------------------------------------------
# Single-trajectory helpers — used by the "Single Projected Trajectory" sections
# of the Streamlit explorer. Each returns one trajectory generated by the
# corresponding C++ projector, packed into a dataclass that the existing
# plot_trajectory_with_footprints helper can consume.
# ----------------------------------------------------------------------------


def single_bicycle_trajectory(
    wheelbase: float,
    track_width: float,
    wheel_radius: float,
    initial_steering_angle_rad: float,
    target_steering_angle_rad: float,
    steering_rate_rad_s: float,
    drive_velocity: float,
    duration: float,
    time_step: float = 0.02,
    min_steering_angle_rad: float | None = None,
    max_steering_angle_rad: float | None = None,
) -> BicycleTrajectory:
    """Ramp a bicycle steering angle from `initial` toward `target` at `rate` rad/s
    over `duration` seconds, returning the resulting trajectory.

    Default min/max bracket `[min(initial, target), max(initial, target)]` so the
    clamp never fires unless the caller explicitly sets bounds.
    """
    if min_steering_angle_rad is None:
        min_steering_angle_rad = min(initial_steering_angle_rad, target_steering_angle_rad)
    if max_steering_angle_rad is None:
        max_steering_angle_rad = max(initial_steering_angle_rad, target_steering_angle_rad)

    model = BicycleModel(wheelbase, track_width, wheel_radius)
    projector = BicycleProjector(model, min_steering_angle_rad, max_steering_angle_rad)
    states = projector.project(
        horizon_s=duration,
        dt_s=time_step,
        initial_pose=Pose2D(),
        initial_steering_angle_rad=initial_steering_angle_rad,
        target_steering_angle_rad=target_steering_angle_rad,
        steering_rate_rad_s=steering_rate_rad_s,
        linear_velocity_m_s=drive_velocity,
    )
    time_arr, x_arr, y_arr, theta_arr, _omega = _states_to_arrays(states)
    steering_series = np.asarray([s.steering_angle_rad for s in states])
    return BicycleTrajectory(
        time=time_arr,
        x=x_arr,
        y=y_arr,
        theta=theta_arr,
        linear_velocity=drive_velocity,
        angular_velocity=states[0].angular_velocity_rad_s,
        drive_velocity=drive_velocity,
        steering_angle=target_steering_angle_rad,
        turning_radius=states[-1].steering_state.turning_radius_m,
        steering_angle_series=steering_series,
    )


def single_articulated_trajectory(
    articulation_to_front: float,
    articulation_to_rear: float,
    front_track: float,
    rear_track: float,
    front_wheel_radius: float,
    rear_wheel_radius: float,
    initial_articulation_angle_rad: float,
    target_articulation_angle_rad: float,
    articulation_rate_rad_s: float,
    drive_velocity: float,
    duration: float,
    time_step: float = 0.02,
    min_articulation_angle_rad: float | None = None,
    max_articulation_angle_rad: float | None = None,
) -> ArticulatedTrajectory:
    """Ramp an articulation angle from `initial` toward `target` at `rate` rad/s."""
    if min_articulation_angle_rad is None:
        min_articulation_angle_rad = min(initial_articulation_angle_rad, target_articulation_angle_rad)
    if max_articulation_angle_rad is None:
        max_articulation_angle_rad = max(initial_articulation_angle_rad, target_articulation_angle_rad)

    model = ArticulatedModel(
        articulation_to_front, articulation_to_rear,
        front_track, rear_track,
        front_wheel_radius, rear_wheel_radius,
    )
    projector = ArticulatedProjector(model, min_articulation_angle_rad, max_articulation_angle_rad)
    states = projector.project(
        horizon_s=duration,
        dt_s=time_step,
        initial_pose=Pose2D(),
        initial_articulation_angle_rad=initial_articulation_angle_rad,
        target_articulation_angle_rad=target_articulation_angle_rad,
        articulation_rate_rad_s=articulation_rate_rad_s,
        linear_velocity_m_s=drive_velocity,
    )
    time_arr, x_arr, y_arr, theta_arr, _omega = _states_to_arrays(states)
    articulation_series = np.asarray([s.articulation_angle_rad for s in states])
    return ArticulatedTrajectory(
        time=time_arr,
        x=x_arr,
        y=y_arr,
        theta=theta_arr,
        linear_velocity=drive_velocity,
        angular_velocity=states[0].angular_velocity_rad_s,
        drive_velocity=drive_velocity,
        articulation_angle=target_articulation_angle_rad,
        turning_radius=states[-1].vehicle_state.front_axle_turning_radius_m,
        articulation_angle_series=articulation_series,
    )


def single_differential_trajectory(
    wheel_radius: float,
    track_width: float,
    initial_linear_velocity: float,
    initial_angular_velocity: float,
    target_linear_velocity: float,
    target_angular_velocity: float,
    linear_acceleration: float,
    angular_acceleration: float,
    duration: float,
    time_step: float = 0.02,
    min_linear_velocity: float | None = None,
    max_linear_velocity: float | None = None,
    min_angular_velocity: float | None = None,
    max_angular_velocity: float | None = None,
) -> DifferentialTrajectory:
    """Ramp diff-drive body command (v, omega) from initial toward target at the
    given accelerations, integrating pose forward over `duration`.
    """
    if min_linear_velocity is None:
        min_linear_velocity = min(initial_linear_velocity, target_linear_velocity)
    if max_linear_velocity is None:
        max_linear_velocity = max(initial_linear_velocity, target_linear_velocity)
    if min_angular_velocity is None:
        min_angular_velocity = min(initial_angular_velocity, target_angular_velocity)
    if max_angular_velocity is None:
        max_angular_velocity = max(initial_angular_velocity, target_angular_velocity)

    model = DifferentialDriveModel(wheel_radius, track_width)
    projector = DifferentialDriveProjector(
        model,
        min_linear_velocity, max_linear_velocity,
        min_angular_velocity, max_angular_velocity,
    )
    states = projector.project(
        horizon_s=duration,
        dt_s=time_step,
        initial_pose=Pose2D(),
        initial_linear_velocity_m_s=initial_linear_velocity,
        initial_angular_velocity_rad_s=initial_angular_velocity,
        target_linear_velocity_m_s=target_linear_velocity,
        target_angular_velocity_rad_s=target_angular_velocity,
        linear_acceleration_m_s2=linear_acceleration,
        angular_acceleration_rad_s2=angular_acceleration,
    )
    time_arr, x_arr, y_arr, theta_arr, _omega = _states_to_arrays(states)
    final_wheels = states[-1].wheel_velocities
    return DifferentialTrajectory(
        time=time_arr,
        x=x_arr,
        y=y_arr,
        theta=theta_arr,
        linear_velocity=states[-1].linear_velocity_m_s,
        angular_velocity=states[-1].angular_velocity_rad_s,
        left_wheel=final_wheels.left_wheel_velocity_rad_s,
        right_wheel=final_wheels.right_wheel_velocity_rad_s,
        base_wheel_velocity=(
            final_wheels.left_wheel_velocity_rad_s + final_wheels.right_wheel_velocity_rad_s
        ) / 2.0,
    )
