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

from .config import (
    FRONT_OVERHANG,
    HEADING_ARROW_LENGTH,
    HEADING_ARROW_WIDTH,
    LATTICE_CONFIG,
    REAR_AXLE_POSITION,
)
from .types import AnyTrajectory, ArticulatedTrajectory, BicycleTrajectory, DifferentialTrajectory


def _get_traj_attr(trajectory: AnyTrajectory, key: str) -> float:
    """Get trajectory attribute by key name."""
    return getattr(trajectory, key)


def plot_vehicle_footprint(
    ax: plt.Axes,
    x: float,
    y: float,
    theta: float,
    length: float = 1.0,
    width: float = 0.5,
    color: str = 'blue',
    alpha: float = 0.5,
) -> None:
    """Draw a vehicle footprint (rectangle) at a given pose.

    The vehicle is centered on the rear axle position, with REAR_AXLE_POSITION
    controlling how far back the rear is, and FRONT_OVERHANG controlling
    how far forward the front extends.
    """
    corners_local = np.array([
        [-length * REAR_AXLE_POSITION, -width / 2],
        [length * FRONT_OVERHANG, -width / 2],
        [length * FRONT_OVERHANG, width / 2],
        [-length * REAR_AXLE_POSITION, width / 2],
        [-length * REAR_AXLE_POSITION, -width / 2],
    ])

    cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
    corners_world = (rotation_matrix @ corners_local.T).T + np.array([x, y])

    ax.fill(corners_world[:, 0], corners_world[:, 1], color=color, alpha=alpha, edgecolor=color, linewidth=0.5)

    arrow_length = length * HEADING_ARROW_LENGTH
    ax.arrow(
        x,
        y,
        arrow_length * np.cos(theta),
        arrow_length * np.sin(theta),
        head_width=width * HEADING_ARROW_WIDTH,
        head_length=length * 0.1,
        fc=color,
        ec=color,
        alpha=min(1.0, alpha + 0.3),
    )


def plot_articulated_footprint(
    ax: plt.Axes,
    x_front_axle: float,
    y_front_axle: float,
    theta: float,
    articulation_angle: float,
    front_length: float,
    rear_length: float,
    front_width: float,
    rear_width: float,
    color: str = 'blue',
    alpha: float = 0.5,
) -> None:
    """Draw an articulated vehicle footprint (two connected rectangles) at a given pose.

    Args:
        x_front_axle, y_front_axle: Position of the front axle center
        theta: Heading of the front body
        articulation_angle: Angle between front and rear bodies (positive = left turn)
        front_length: Distance from articulation joint to front axle (L_f)
        rear_length: Distance from articulation joint to rear axle (L_r)
        front_width, rear_width: Track widths
        color: Fill color
        alpha: Transparency
    """
    # Compute articulation joint position
    joint_x = x_front_axle - front_length * np.cos(theta)
    joint_y = y_front_axle - front_length * np.sin(theta)

    # Front body corners
    front_corners_local = np.array([
        [0, -front_width / 2],
        [front_length, -front_width / 2],
        [front_length, front_width / 2],
        [0, front_width / 2],
        [0, -front_width / 2],
    ])

    cos_front, sin_front = np.cos(theta), np.sin(theta)
    rotation_front = np.array([[cos_front, -sin_front], [sin_front, cos_front]])
    front_corners_world = (rotation_front @ front_corners_local.T).T + np.array([joint_x, joint_y])
    ax.fill(
        front_corners_world[:, 0], front_corners_world[:, 1], color=color, alpha=alpha, edgecolor=color, linewidth=0.5
    )

    # Rear body
    rear_theta = theta - articulation_angle
    rear_corners_local = np.array([
        [-rear_length, -rear_width / 2],
        [0, -rear_width / 2],
        [0, rear_width / 2],
        [-rear_length, rear_width / 2],
        [-rear_length, -rear_width / 2],
    ])

    cos_rear, sin_rear = np.cos(rear_theta), np.sin(rear_theta)
    rotation_rear = np.array([[cos_rear, -sin_rear], [sin_rear, cos_rear]])
    rear_corners_world = (rotation_rear @ rear_corners_local.T).T + np.array([joint_x, joint_y])
    ax.fill(
        rear_corners_world[:, 0],
        rear_corners_world[:, 1],
        color=color,
        alpha=alpha * 0.8,
        edgecolor=color,
        linewidth=0.5,
    )

    # Articulation joint marker
    ax.plot(joint_x, joint_y, 'o', color=color, markersize=4, alpha=min(1.0, alpha + 0.3))

    # Heading arrow
    arrow_length = front_length * 0.4
    ax.arrow(
        x_front_axle,
        y_front_axle,
        arrow_length * np.cos(theta),
        arrow_length * np.sin(theta),
        head_width=front_width * 0.2,
        head_length=front_length * 0.08,
        fc=color,
        ec=color,
        alpha=min(1.0, alpha + 0.3),
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

    all_angles = sorted(set(_get_traj_attr(trajectory, angle_key) for trajectory in trajectories))
    all_velocities = sorted(set(_get_traj_attr(trajectory, vel_key) for trajectory in trajectories))

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
        if any(abs(_get_traj_attr(trajectory, angle_key) - angle) < 1e-6 for angle in selected_angles)
        and any(abs(_get_traj_attr(trajectory, vel_key) - velocity) < 1e-6 for velocity in selected_velocities)
    ]

    selected.sort(key=lambda trajectory: _get_traj_attr(trajectory, angle_key))
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

    Args:
        trajectories: List of trajectory dataclasses (already filtered/selected)
        model_type: "Differential Drive", "Bicycle", or "Articulated"
        model_params: Dict with model dimensions
        num_footprints: Number of footprints per trajectory

    Returns:
        matplotlib Figure
    """
    fig, ax = plt.subplots(figsize=(10, 8), layout='constrained')

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
                plot_articulated_footprint(
                    ax,
                    trajectory.x[footprint_index],
                    trajectory.y[footprint_index],
                    trajectory.theta[footprint_index],
                    articulation_angle=articulated.articulation_angle,
                    front_length=model_params['front_length'],
                    rear_length=model_params['rear_length'],
                    front_width=model_params['front_width'],
                    rear_width=model_params['rear_width'],
                    color=color,
                    alpha=footprint_alpha,
                )
            else:
                plot_vehicle_footprint(
                    ax,
                    trajectory.x[footprint_index],
                    trajectory.y[footprint_index],
                    trajectory.theta[footprint_index],
                    length=model_params['length'],
                    width=model_params['width'],
                    color=color,
                    alpha=footprint_alpha,
                )

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Trajectory with Vehicle Footprints ({model_type})')
    ax.set_aspect('equal')
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
    fig, axes = plt.subplots(1, num_columns, figsize=(5 * num_columns, 5), layout='constrained')

    if num_columns == 1:
        axes = [axes]

    color_values = [_get_traj_attr(trajectory, config.color_key) for trajectory in trajectories]
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
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linewidth=0.5)
        ax.axvline(x=0, color='k', linewidth=0.5)

        for trajectory in trajectories:
            if abs(_get_traj_attr(trajectory, config.group_key) - group_value) < 0.001:
                raw_color_value = _get_traj_attr(trajectory, config.color_key)
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
        ax_right.legend()
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
        ax_right.legend()
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
