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
"""Configuration constants and data structures for kinematic exploration."""

from __future__ import annotations

from dataclasses import dataclass

# Vehicle footprint positioning (fraction of length)
REAR_AXLE_POSITION = 0.3  # How far back the rear axle is from center
FRONT_OVERHANG = 0.7  # How far forward the front extends from center
HEADING_ARROW_LENGTH = 0.4  # Arrow length as fraction of vehicle length
HEADING_ARROW_WIDTH = 0.3  # Arrow head width as fraction of vehicle width


@dataclass
class LatticeConfig:
    """Configuration for plotting trajectory lattices."""

    group_key: str
    group_label: str
    group_unit: str
    color_key: str
    color_label: str
    color_unit: str
    color_is_angle: bool = False
    angle_key: str = ''  # Key for angle in trajectory selection
    vel_key: str = ''  # Key for velocity in trajectory selection


LATTICE_CONFIG: dict[str, LatticeConfig] = {
    'Differential Drive': LatticeConfig(
        group_key='base_wheel_velocity',
        group_label='Base Wheel Velocity',
        group_unit='rad/s',
        color_key='angular_velocity',
        color_label='Angular Velocity',
        color_unit='rad/s',
        color_is_angle=False,
        angle_key='angular_velocity',
        vel_key='base_wheel_velocity',
    ),
    'Bicycle': LatticeConfig(
        group_key='drive_velocity',
        group_label='Drive Velocity',
        group_unit='m/s',
        color_key='steering_angle',
        color_label='Steering Angle',
        color_unit='deg',
        color_is_angle=True,
        angle_key='steering_angle',
        vel_key='drive_velocity',
    ),
    'Articulated': LatticeConfig(
        group_key='drive_velocity',
        group_label='Drive Velocity',
        group_unit='m/s',
        color_key='articulation_angle',
        color_label='Articulation Angle',
        color_unit='deg',
        color_is_angle=True,
        angle_key='articulation_angle',
        vel_key='drive_velocity',
    ),
}


KINEMATIC_EQUATIONS = {
    'Differential Drive': {
        'title': 'Differential Drive Kinematics',
        'equations': [
            r'v = \frac{r}{2}(\omega_L + \omega_R)',
            r'\omega = \frac{r}{W}(\omega_R - \omega_L)',
        ],
        'variables': r'$r$ = wheel radius, $W$ = track width, $\omega_L, \omega_R$ = wheel velocities',
    },
    'Bicycle': {
        'title': 'Bicycle Model Kinematics',
        'equations': [
            r'\omega = \frac{v \tan(\delta)}{L}',
            r'R = \frac{L}{\tan(\delta)}',
        ],
        'variables': r'$L$ = wheelbase, $\delta$ = steering angle, $R$ = turning radius',
    },
    'Articulated': {
        'title': 'Articulated Vehicle Kinematics (Corke & Ridley)',
        'equations': [
            r'\omega = \frac{v \sin\gamma + L_r \dot{\gamma}}{L_f \cos\gamma + L_r}',
            r'R_f = \frac{L_f \cos\gamma + L_r}{\sin\gamma}',
        ],
        'variables': r'$L_f, L_r$ = front/rear distances to articulation joint, $\gamma$ = articulation angle, $\dot{\gamma}$ = articulation rate (currently 0)',
        'reference': "Corke & Ridley, IEEE IO'A 2001",
    },
}

TRAJECTORY_EQUATIONS = r"""
**Trajectory Integration (Euler method):**
$$\dot{x} = v \cos(\theta), \quad \dot{y} = v \sin(\theta), \quad \dot{\theta} = \omega$$
"""

TRAJECTORY_EQUATIONS_RK4 = r"""
**Trajectory Integration (RK4 method):**
$$\dot{x} = v \cos(\theta), \quad \dot{y} = v \sin(\theta), \quad \dot{\theta} = \omega$$
"""
