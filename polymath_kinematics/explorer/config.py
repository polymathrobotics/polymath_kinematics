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

# Footprint overhang slider defaults (metres, measured beyond the reference axle/centre).

# Bicycle: pose reference is the rear axle, so front_overhang_m = wheelbase + front overhang.
DEFAULT_BICYCLE_FRONT_OVERHANG_M = 0.6
DEFAULT_BICYCLE_REAR_OVERHANG_M = 0.5

# Differential drive: pose reference is the body centre, so the overhangs pass through directly.
DEFAULT_DIFFERENTIAL_FRONT_OVERHANG_M = 0.4
DEFAULT_DIFFERENTIAL_REAR_OVERHANG_M = 0.4

# Articulated: base_link is the articulation joint; joint-to-bumper distances are axle distance +
# overhang, so a positive overhang makes the body extend behind the rear axle (counterweight) and
# ahead of the front axle (bucket).
DEFAULT_ARTICULATED_FRONT_OVERHANG_M = 1.0
DEFAULT_ARTICULATED_REAR_OVERHANG_M = 0.8


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
        'variables': r'$L_f, L_r$ = front/rear distances to articulation joint, $\gamma$ = articulation angle, $\dot{\gamma}$ = articulation rate',
        'reference': "Corke & Ridley, IEEE IO'A 2001",
    },
}

TRAJECTORY_EQUATIONS = r"""
**Trajectory Integration (Euler method):**
$$\dot{x} = v \cos(\theta), \quad \dot{y} = v \sin(\theta), \quad \dot{\theta} = \omega$$
"""
