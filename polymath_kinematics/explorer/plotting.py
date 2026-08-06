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
"""Plotting functions for kinematic visualization.

All functions return matplotlib Figure objects and have no streamlit dependencies.
"""

from __future__ import annotations

import matplotlib.pyplot as plt
import numpy as np

from .config import LATTICE_CONFIG
from .types import AnyTrajectory, ArticulatedTrajectory, BicycleTrajectory, DifferentialTrajectory


def get_traj_attr(trajectory: AnyTrajectory, key: str) -> float:
    """Get trajectory attribute by key name."""
    return getattr(trajectory, key)


def plot_vehicle_footprint(
    ax: plt.Axes,
    x: float,
    y: float,
    theta: float,
    body_corners: np.ndarray,
    color: str = 'blue',
    alpha: float = 0.5,
    steering_angle: float | None = None,
    front_axle_offset_m: float | None = None,
    track_width_m: float | None = None,
    wheel_length_frac: float = 0.18,
    wheel_width_frac: float = 0.08,
) -> None:
    """Draw a single-body vehicle footprint at a given pose.

    ``body_corners`` is the projector's ``footprint``: world-frame corners in the projector's
    order, rear-right, front-right, front-left, rear-left. The heading arrow is derived from it.

    ``steering_angle`` (radians) draws two front-wheel indicators at ``front_axle_offset_m``
    ahead of the pose reference, spaced by ``track_width_m``. Pass ``None`` for models without
    steered wheels (e.g. differential drive).
    """
    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])

    body = np.asarray(body_corners, dtype=float)
    ax.fill(body[:, 0], body[:, 1], color=color, alpha=alpha, edgecolor=color, linewidth=0.5)

    # Heading arrow spans the body: rear-edge midpoint -> front-edge midpoint.
    rear_mid = (body[0] + body[3]) / 2.0
    front_mid = (body[1] + body[2]) / 2.0
    heading_vec = front_mid - rear_mid
    body_length = float(np.hypot(*heading_vec))
    body_width = float(np.hypot(*(body[2] - body[1])))
    ax.arrow(
        rear_mid[0],
        rear_mid[1],
        heading_vec[0],
        heading_vec[1],
        head_width=body_width * 0.3,
        head_length=body_length * 0.1,
        fc=color,
        ec=color,
        alpha=min(1.0, alpha + 0.3),
        length_includes_head=True,
    )

    if steering_angle is not None and front_axle_offset_m is not None and track_width_m is not None:
        # Front-wheel indicators: rectangles at the front axle, offset left/right by half the
        # track width, each rotated by steering_angle in the body frame.
        wheel_l = body_length * wheel_length_frac
        wheel_w = body_width * wheel_width_frac
        wheel_corners_centered = np.array([
            [-wheel_l / 2.0, -wheel_w / 2.0],
            [+wheel_l / 2.0, -wheel_w / 2.0],
            [+wheel_l / 2.0, +wheel_w / 2.0],
            [-wheel_l / 2.0, +wheel_w / 2.0],
            [-wheel_l / 2.0, -wheel_w / 2.0],
        ])
        cos_s, sin_s = np.cos(steering_angle), np.sin(steering_angle)
        wheel_rotation = np.array([[cos_s, -sin_s], [sin_s, cos_s]])
        wheel_corners_steered = (wheel_rotation @ wheel_corners_centered.T).T  # (5, 2)
        for lateral_offset in (-track_width_m / 2.0, +track_width_m / 2.0):
            corners_body = wheel_corners_steered + np.array([front_axle_offset_m, lateral_offset])
            corners_in_world = (rotation_matrix @ corners_body.T).T + np.array([x, y])
            ax.fill(
                corners_in_world[:, 0],
                corners_in_world[:, 1],
                color=color,
                alpha=min(1.0, alpha + 0.2),
                edgecolor=color,
                linewidth=0.5,
            )


def plot_articulated_footprint(
    ax: plt.Axes,
    front_corners: np.ndarray,
    rear_corners: np.ndarray,
    color: str = 'blue',
    alpha: float = 0.5,
) -> None:
    """Draw an articulated vehicle footprint (two connected rectangles).

    Corners come from the projector's ``front_footprint`` / ``rear_footprint``, which already
    carry the rear-segment anchoring about the joint. Front body is
    [joint-right, bumper-right, bumper-left, joint-left], so the joint is mid(0, 3); rear body is
    [bumper-right, joint-right, joint-left, bumper-left]. Joint marker and arrow follow from those.
    """
    fc = np.asarray(front_corners, dtype=float)
    rc = np.asarray(rear_corners, dtype=float)

    ax.fill(fc[:, 0], fc[:, 1], color=color, alpha=alpha, edgecolor=color, linewidth=0.5)
    ax.fill(rc[:, 0], rc[:, 1], color=color, alpha=alpha * 0.8, edgecolor=color, linewidth=0.5)

    joint_point = (fc[0] + fc[3]) / 2.0
    front_bumper_mid = (fc[1] + fc[2]) / 2.0
    front_width = float(np.hypot(*(fc[2] - fc[1])))

    ax.plot(joint_point[0], joint_point[1], 'o', color=color, markersize=4, alpha=min(1.0, alpha + 0.3))

    heading_vec = front_bumper_mid - joint_point
    front_length = float(np.hypot(*heading_vec))
    ax.arrow(
        joint_point[0],
        joint_point[1],
        heading_vec[0],
        heading_vec[1],
        head_width=front_width * 0.2,
        head_length=front_length * 0.08 if front_length else 0.05,
        fc=color,
        ec=color,
        alpha=min(1.0, alpha + 0.3),
        length_includes_head=True,
    )


def select_symmetric_trajectories(
    trajectories: list[AnyTrajectory],
    model_type: str,
    num_angles: int = 5,
    num_velocities: int = 1,
) -> list[AnyTrajectory]:
    """Select a symmetric subset of trajectories for visualization.

    Args:
        trajectories: Full list of trajectory dataclasses
        model_type: Model type to determine which keys to use
        num_angles: Number of steering/articulation angles (should be odd for symmetry)
        num_velocities: Number of velocities to include

    Returns:
        Filtered list of trajectories, symmetrically selected
    """
    if not trajectories:
        return []

    config = LATTICE_CONFIG.get(model_type)
    if config is None:
        return trajectories[:5]

    angle_key = config.angle_key
    vel_key = config.vel_key

    all_angles = sorted(set(get_traj_attr(trajectory, angle_key) for trajectory in trajectories))
    all_velocities = sorted(set(get_traj_attr(trajectory, vel_key) for trajectory in trajectories))

    # Select symmetric angles
    if len(all_angles) <= num_angles:
        selected_angles = set(all_angles)
    else:
        indices = np.linspace(0, len(all_angles) - 1, num_angles, dtype=int)
        selected_angles = {all_angles[i] for i in indices}

    # Select velocities
    if len(all_velocities) <= num_velocities:
        selected_velocities = set(all_velocities)
    else:
        indices = np.linspace(0, len(all_velocities) - 1, num_velocities, dtype=int)
        selected_velocities = {all_velocities[i] for i in indices}

    selected = [
        trajectory
        for trajectory in trajectories
        if any(abs(get_traj_attr(trajectory, angle_key) - angle) < 1e-6 for angle in selected_angles)
        and any(abs(get_traj_attr(trajectory, vel_key) - velocity) < 1e-6 for velocity in selected_velocities)
    ]

    selected.sort(key=lambda trajectory: get_traj_attr(trajectory, angle_key))
    return selected


def _build_legend_label(trajectory: AnyTrajectory, model_type: str) -> str:
    """Build legend label for a trajectory based on model type."""
    if model_type == 'Articulated':
        articulated: ArticulatedTrajectory = trajectory  # type: ignore[assignment]
        return f'y={np.degrees(articulated.articulation_angle):+.0f} deg, v={articulated.drive_velocity:.1f}'
    elif model_type == 'Bicycle':
        bicycle: BicycleTrajectory = trajectory  # type: ignore[assignment]
        return f'd={np.degrees(bicycle.steering_angle):+.0f} deg, v={bicycle.drive_velocity:.1f}'
    elif model_type == 'Differential Drive':
        differential: DifferentialTrajectory = trajectory  # type: ignore[assignment]
        return f'w={differential.angular_velocity:+.1f}, base={differential.base_wheel_velocity:.0f}'
    return ''


def plot_trajectory_with_footprints(
    trajectories: list[AnyTrajectory],
    model_type: str,
    model_params: dict,
    num_footprints: int = 5,
) -> plt.Figure:
    """Plot selected trajectories with vehicle footprints at intervals.

    A trajectory with no footprint series (projector given no body dims) still has its path
    drawn; only the overlay is skipped.

    Args:
        trajectories: List of trajectory dataclasses (already filtered/selected)
        model_type: "Differential Drive", "Bicycle", or "Articulated"
        model_params: Bicycle only — ``wheelbase`` and ``track_width`` for the steered-wheel
            indicators. Unused for the other models.
        num_footprints: Number of footprints per trajectory

    Returns:
        matplotlib Figure
    """
    fig, ax = plt.subplots(figsize=(16, 9), layout='constrained')

    colormap = plt.cm.tab10
    legend_handles = []
    legend_labels = []

    for index, trajectory in enumerate(trajectories):
        color = colormap(index % 10)

        (line,) = ax.plot(trajectory.x, trajectory.y, color=color, linewidth=2, alpha=0.7)
        legend_handles.append(line)
        legend_labels.append(_build_legend_label(trajectory, model_type))

        num_points = len(trajectory)
        footprint_indices = np.linspace(0, num_points - 1, num_footprints, dtype=int)

        for footprint_num, footprint_index in enumerate(footprint_indices):
            footprint_alpha = 0.3 + 0.5 * (footprint_num / max(1, num_footprints - 1))

            if model_type == 'Articulated':
                articulated: ArticulatedTrajectory = trajectory  # type: ignore[assignment]
                if articulated.front_footprint_series is None or articulated.rear_footprint_series is None:
                    continue
                plot_articulated_footprint(
                    ax,
                    articulated.front_footprint_series[footprint_index],
                    articulated.rear_footprint_series[footprint_index],
                    color=color,
                    alpha=footprint_alpha,
                )
            elif model_type == 'Bicycle':
                bicycle: BicycleTrajectory = trajectory  # type: ignore[assignment]
                if bicycle.footprint_series is None:
                    continue
                if bicycle.steering_angle_series is not None:
                    sample_steering_angle = float(bicycle.steering_angle_series[footprint_index])
                else:
                    sample_steering_angle = bicycle.steering_angle
                plot_vehicle_footprint(
                    ax,
                    trajectory.x[footprint_index],
                    trajectory.y[footprint_index],
                    trajectory.theta[footprint_index],
                    bicycle.footprint_series[footprint_index],
                    color=color,
                    alpha=footprint_alpha,
                    steering_angle=sample_steering_angle,
                    front_axle_offset_m=model_params['wheelbase'],
                    track_width_m=model_params['track_width'],
                )
            else:  # Differential Drive — no steered wheels.
                if trajectory.footprint_series is None:
                    continue
                plot_vehicle_footprint(
                    ax,
                    trajectory.x[footprint_index],
                    trajectory.y[footprint_index],
                    trajectory.theta[footprint_index],
                    trajectory.footprint_series[footprint_index],
                    color=color,
                    alpha=footprint_alpha,
                )

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Trajectory with Vehicle Footprints ({model_type})')
    # Equal data scaling (circles stay circular) but force a 16:9 landscape box; the view
    # limits expand to fill the box rather than distorting the geometry.
    ax.set_aspect('equal')
    ax.set_box_aspect(9 / 16)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linewidth=0.5)
    ax.axvline(x=0, color='k', linewidth=0.5)

    if legend_handles:
        title = {
            'Articulated': 'Articulation',
            'Bicycle': 'Steering',
            'Differential Drive': 'Angular Vel',
        }.get(model_type, '')
        ax.legend(legend_handles, legend_labels, loc='upper right', title=title)

    return fig


def plot_lattice(
    trajectories: list[AnyTrajectory],
    model_type: str,
    group_values: list[float],
) -> plt.Figure:
    """Plot trajectory lattice grouped by velocity, colored by steering/articulation angle.

    Args:
        trajectories: List of trajectory dataclasses
        model_type: "Differential Drive", "Bicycle", or "Articulated"
        group_values: List of velocity values to create subplots for

    Returns:
        matplotlib Figure
    """
    config = LATTICE_CONFIG[model_type]

    num_columns = len(group_values)
    # 16:9 landscape; grow width for the (rare) multi-column case.
    fig_width = max(16, 8 * num_columns)
    fig, axes = plt.subplots(1, num_columns, figsize=(fig_width, 9), layout='constrained')

    if num_columns == 1:
        axes = [axes]

    color_values = [get_traj_attr(trajectory, config.color_key) for trajectory in trajectories]
    if config.color_is_angle:
        color_values_display = [np.degrees(value) for value in color_values]
    else:
        color_values_display = color_values

    colormap = plt.cm.coolwarm
    normalizer = plt.Normalize(vmin=min(color_values_display), vmax=max(color_values_display))

    for ax, group_value in zip(axes, group_values):
        ax.set_title(f'{config.group_label} = {group_value:.1f} {config.group_unit}')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        # Equal data scaling (circles stay circular) with a 16:9 landscape box.
        ax.set_aspect('equal')
        ax.set_box_aspect(9 / 16)
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linewidth=0.5)
        ax.axvline(x=0, color='k', linewidth=0.5)

        for trajectory in trajectories:
            if abs(get_traj_attr(trajectory, config.group_key) - group_value) < 0.001:
                raw_color_value = get_traj_attr(trajectory, config.color_key)
                color_value = np.degrees(raw_color_value) if config.color_is_angle else raw_color_value
                color = colormap(normalizer(color_value))
                ax.plot(trajectory.x, trajectory.y, color=color, linewidth=1.5, alpha=0.8)
                ax.plot(trajectory.x[-1], trajectory.y[-1], 'o', color=color, markersize=3)

    scalar_mappable = plt.cm.ScalarMappable(cmap=colormap, norm=normalizer)
    scalar_mappable.set_array([])
    colorbar = fig.colorbar(scalar_mappable, ax=axes, orientation='horizontal', fraction=0.05, pad=0.12)
    colorbar.set_label(f'{config.color_label} ({config.color_unit})')

    return fig


def plot_analysis(
    trajectories: list[AnyTrajectory],
    model_type: str,
    group_values: list[float],
) -> plt.Figure:
    """Plot kinematic analysis for all velocities in the lattice.

    Args:
        trajectories: List of trajectory dataclasses
        model_type: "Differential Drive", "Bicycle", or "Articulated"
        group_values: List of velocity values to analyze

    Returns:
        matplotlib Figure with two subplots
    """
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), layout='constrained')

    if model_type == 'Articulated':
        ax_left = axes[0]
        for velocity in group_values:
            angles = []
            angular_velocities = []
            for trajectory in trajectories:
                articulated: ArticulatedTrajectory = trajectory  # type: ignore[assignment]
                if abs(articulated.drive_velocity - velocity) < 0.001:
                    angles.append(np.degrees(articulated.articulation_angle))
                    angular_velocities.append(articulated.angular_velocity)
            if angles:
                ax_left.plot(angles, angular_velocities, 'o-', label=f'v={velocity:.1f} m/s', markersize=5)
        ax_left.set_xlabel('Articulation Angle (deg)')
        ax_left.set_ylabel('Angular Velocity (rad/s)')
        ax_left.set_title('Articulation -> Angular Velocity')
        ax_left.legend()
        ax_left.grid(True, alpha=0.3)

        ax_right = axes[1]
        radii = []
        angles = []
        for trajectory in trajectories:
            articulated: ArticulatedTrajectory = trajectory  # type: ignore[assignment]
            if abs(articulated.articulation_angle) > 0.00001:
                radii.append(abs(articulated.turning_radius))
                angles.append(np.degrees(abs(articulated.articulation_angle)))
        if angles:
            ax_right.plot(angles, radii, 'o-', markersize=5)

        ax_right.set_xlabel('|Articulation Angle| (deg)')
        ax_right.set_ylabel('Turning Radius (m)')
        ax_right.set_title('Turning Radius vs Articulation')
        # Single unlabeled series — no legend.
        ax_right.grid(True, alpha=0.3)
        ax_right.set_ylim(bottom=0)

    elif model_type == 'Bicycle':
        ax_left = axes[0]
        for velocity in group_values:
            angles = []
            angular_velocities = []
            for trajectory in trajectories:
                bicycle: BicycleTrajectory = trajectory  # type: ignore[assignment]
                if abs(bicycle.drive_velocity - velocity) < 0.001:
                    angles.append(np.degrees(bicycle.steering_angle))
                    angular_velocities.append(bicycle.angular_velocity)
            if angles:
                ax_left.plot(angles, angular_velocities, 'o-', label=f'v={velocity:.1f} m/s', markersize=5)
        ax_left.set_xlabel('Steering Angle (deg)')
        ax_left.set_ylabel('Angular Velocity (rad/s)')
        ax_left.set_title('Steering -> Angular Velocity')
        ax_left.legend()
        ax_left.grid(True, alpha=0.3)

        ax_right = axes[1]

        radii = []
        angles = []
        for trajectory in trajectories:
            bicycle: BicycleTrajectory = trajectory  # type: ignore[assignment]
            if abs(bicycle.steering_angle) > 0.00001:
                radii.append(abs(bicycle.turning_radius))
                angles.append(np.degrees(abs(bicycle.steering_angle)))

        if angles:
            ax_right.plot(angles, radii, 'o-', markersize=5)

        ax_right.set_xlabel('|Steering Angle| (deg)')
        ax_right.set_ylabel('Turning Radius (m)')
        ax_right.set_title('Turning Radius vs Steering')
        # Single unlabeled series — no legend.
        ax_right.grid(True, alpha=0.3)
        ax_right.set_ylim(bottom=0)

    elif model_type == 'Differential Drive':
        ax_left = axes[0]
        base_velocity = group_values[len(group_values) // 2]
        left_wheel_velocities = []
        right_wheel_velocities = []
        angular_velocities = []
        for trajectory in trajectories:
            differential: DifferentialTrajectory = trajectory  # type: ignore[assignment]
            if abs(differential.base_wheel_velocity - base_velocity) < 0.001:
                left_wheel_velocities.append(differential.left_wheel)
                right_wheel_velocities.append(differential.right_wheel)
                angular_velocities.append(differential.angular_velocity)
        if left_wheel_velocities:
            ax_left.plot(angular_velocities, left_wheel_velocities, 'o-', label='Left wheel', markersize=5)
            ax_left.plot(angular_velocities, right_wheel_velocities, 's-', label='Right wheel', markersize=5)
        ax_left.set_xlabel('Angular Velocity (rad/s)')
        ax_left.set_ylabel('Wheel Velocity (rad/s)')
        ax_left.set_title(f'Wheel Velocities (base={base_velocity:.0f} rad/s)')
        ax_left.legend()
        ax_left.grid(True, alpha=0.3)

        ax_right = axes[1]
        for base_velocity in group_values:
            linear_velocities = []
            angular_velocities_list = []
            for trajectory in trajectories:
                differential: DifferentialTrajectory = trajectory  # type: ignore[assignment]
                if abs(differential.base_wheel_velocity - base_velocity) < 0.001:
                    linear_velocities.append(differential.linear_velocity)
                    angular_velocities_list.append(differential.angular_velocity)
            if linear_velocities:
                ax_right.plot(
                    angular_velocities_list,
                    linear_velocities,
                    'o-',
                    label=f'base={base_velocity:.0f} rad/s',
                    markersize=5,
                )
        ax_right.set_xlabel('Angular Velocity (rad/s)')
        ax_right.set_ylabel('Linear Velocity (m/s)')
        ax_right.set_title('Body Velocities')
        ax_right.legend()
        ax_right.grid(True, alpha=0.3)

    return fig
