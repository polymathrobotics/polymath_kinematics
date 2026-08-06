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

Every trajectory comes from the matching C++ projector, so the explorer and the production
kinematics share one forward-simulation codepath. Lattice cells set initial == target to hold
a constant command; the single-trajectory helpers drive initial != target to show the ramp.

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


def _footprint_series(states, attr: str):
    """Stack the per-sample footprint polygons from a projection into an (N, 4, 2) array.

    ``attr`` is the projected-state attribute holding the footprint (``footprint`` for single-body
    models, ``front_footprint`` / ``rear_footprint`` for articulated). Returns ``None`` if any
    sample has an empty footprint (i.e. the projector was given no footprint dimensions), so
    downstream consumers can fall back cleanly.
    """
    corners = []
    for s in states:
        polygon = getattr(s, attr)
        if not polygon:
            return None
        corners.append([[p.x, p.y] for p in polygon])
    return np.asarray(corners, dtype=float)


def generate_lattice_differential(
    wheel_radius: float,
    track_width: float,
    base_wheel_velocities: tuple[float, ...],
    wheel_velocity_diffs: tuple[float, ...],
    duration: float,
    time_step: float = 0.02,
    front_overhang_m: float = 0.0,
    rear_overhang_m: float = 0.0,
    body_width_m: float = 0.0,
) -> list[DifferentialTrajectory]:
    """Generate trajectory lattice by sweeping wheel velocity differences.

    Each wheel-velocity pair becomes a body command, then a constant-command projection.
    """
    model = DifferentialDriveModel(wheel_radius, track_width)

    # Resolve all cells first: the projector bounds must bracket the sweep, or outer cells clamp.
    cells = []
    for base_velocity in base_wheel_velocities:
        for velocity_diff in wheel_velocity_diffs:
            left_wheel = base_velocity - velocity_diff / 2
            right_wheel = base_velocity + velocity_diff / 2
            body_velocity = model.wheel_velocities_to_body_velocity(left_wheel, right_wheel)
            cells.append((
                base_velocity,
                left_wheel,
                right_wheel,
                body_velocity.linear_velocity_m_s,
                body_velocity.angular_velocity_rad_s,
            ))

    if not cells:
        return []

    linear_velocities = [cell[3] for cell in cells]
    angular_velocities = [cell[4] for cell in cells]
    projector = DifferentialDriveProjector(
        model,
        min(linear_velocities),
        max(linear_velocities),
        min(angular_velocities),
        max(angular_velocities),
        front_overhang_m,
        rear_overhang_m,
        body_width_m,
    )

    trajectories: list[DifferentialTrajectory] = []
    for base_velocity, left_wheel, right_wheel, linear_velocity, angular_velocity in cells:
        states = projector.project(
            horizon_s=duration,
            dt_s=time_step,
            initial_pose=Pose2D(),
            initial_linear_velocity_m_s=linear_velocity,
            initial_angular_velocity_rad_s=angular_velocity,
            target_linear_velocity_m_s=linear_velocity,
            target_angular_velocity_rad_s=angular_velocity,
            # initial == target, so the ramp never fires and the acceleration is irrelevant.
            linear_acceleration_m_s2=0.0,
            angular_acceleration_rad_s2=0.0,
        )
        time_arr, x_arr, y_arr, theta_arr, _omega = _states_to_arrays(states)

        trajectories.append(
            DifferentialTrajectory(
                time=time_arr,
                x=x_arr,
                y=y_arr,
                theta=theta_arr,
                linear_velocity=linear_velocity,
                angular_velocity=angular_velocity,
                left_wheel=left_wheel,
                right_wheel=right_wheel,
                base_wheel_velocity=base_velocity,
                footprint_series=_footprint_series(states, 'footprint'),
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
    min_steering_angle_rad: float | None = None,
    max_steering_angle_rad: float | None = None,
    steering_rate_rad_s: float = 0.0,
    front_overhang_m: float = 0.0,
    rear_overhang_m: float = 0.0,
    body_width_m: float = 0.0,
) -> list[BicycleTrajectory]:
    """Generate trajectory lattice by sweeping steering angles.

    Each lattice cell is generated via BicycleProjector.project() with the
    initial steering angle equal to the target, so no ramp occurs and the
    behavior matches a constant-steering sweep. ``steering_rate_rad_s`` is
    exposed for future use; with a non-zero value and ``min/max`` set, the
    projector will ramp toward the target — but a constant-input lattice
    is the typical use.
    """
    angles = list(steering_angles)
    if not angles:
        return []
    min_angle = min_steering_angle_rad if min_steering_angle_rad is not None else min(angles)
    max_angle = max_steering_angle_rad if max_steering_angle_rad is not None else max(angles)

    model = BicycleModel(wheelbase, track_width, wheel_radius)
    projector = BicycleProjector(model, min_angle, max_angle, front_overhang_m, rear_overhang_m, body_width_m)

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
                    footprint_series=_footprint_series(states, 'footprint'),
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
    min_articulation_angle_rad: float | None = None,
    max_articulation_angle_rad: float | None = None,
    articulation_rate_rad_s: float = 0.0,
    front_joint_to_bumper_m: float = 0.0,
    front_body_width_m: float = 0.0,
    rear_joint_to_bumper_m: float = 0.0,
    rear_body_width_m: float = 0.0,
) -> list[ArticulatedTrajectory]:
    """Generate trajectory lattice by sweeping articulation angles.

    See ``generate_lattice_bicycle`` for the projector-based rationale.
    """
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
    projector = ArticulatedProjector(
        model,
        min_angle,
        max_angle,
        front_joint_to_bumper_m,
        front_body_width_m,
        rear_joint_to_bumper_m,
        rear_body_width_m,
    )

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
                    front_footprint_series=_footprint_series(states, 'front_footprint'),
                    rear_footprint_series=_footprint_series(states, 'rear_footprint'),
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
    front_overhang_m: float = 0.0,
    rear_overhang_m: float = 0.0,
    body_width_m: float = 0.0,
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
    projector = BicycleProjector(
        model, min_steering_angle_rad, max_steering_angle_rad, front_overhang_m, rear_overhang_m, body_width_m
    )
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
        footprint_series=_footprint_series(states, 'footprint'),
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
    front_joint_to_bumper_m: float = 0.0,
    front_body_width_m: float = 0.0,
    rear_joint_to_bumper_m: float = 0.0,
    rear_body_width_m: float = 0.0,
) -> ArticulatedTrajectory:
    """Ramp an articulation angle from `initial` toward `target` at `rate` rad/s."""
    if min_articulation_angle_rad is None:
        min_articulation_angle_rad = min(initial_articulation_angle_rad, target_articulation_angle_rad)
    if max_articulation_angle_rad is None:
        max_articulation_angle_rad = max(initial_articulation_angle_rad, target_articulation_angle_rad)

    model = ArticulatedModel(
        articulation_to_front,
        articulation_to_rear,
        front_track,
        rear_track,
        front_wheel_radius,
        rear_wheel_radius,
    )
    projector = ArticulatedProjector(
        model,
        min_articulation_angle_rad,
        max_articulation_angle_rad,
        front_joint_to_bumper_m,
        front_body_width_m,
        rear_joint_to_bumper_m,
        rear_body_width_m,
    )
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
        front_footprint_series=_footprint_series(states, 'front_footprint'),
        rear_footprint_series=_footprint_series(states, 'rear_footprint'),
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
    front_overhang_m: float = 0.0,
    rear_overhang_m: float = 0.0,
    body_width_m: float = 0.0,
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
        min_linear_velocity,
        max_linear_velocity,
        min_angular_velocity,
        max_angular_velocity,
        front_overhang_m,
        rear_overhang_m,
        body_width_m,
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
        base_wheel_velocity=(final_wheels.left_wheel_velocity_rad_s + final_wheels.right_wheel_velocity_rad_s) / 2.0,
        footprint_series=_footprint_series(states, 'footprint'),
    )
