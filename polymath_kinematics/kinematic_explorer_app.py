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
"""Interactive Kinematic Model Explorer - Streamlit UI.

Run with: streamlit run kinematic_explorer_app.py
"""

from __future__ import annotations

import io
import json
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
import streamlit as st

from polymath_kinematics.explorer import (
    KINEMATIC_EQUATIONS,
    LATTICE_CONFIG,
    TRAJECTORY_EQUATIONS,
    AnyTrajectory,
    ArticulatedTrajectory,
    BicycleTrajectory,
    DifferentialTrajectory,
    generate_lattice_articulated,
    generate_lattice_bicycle,
    generate_lattice_differential,
    get_traj_attr,
    plot_analysis,
    plot_lattice,
    plot_trajectory_with_footprints,
    select_symmetric_trajectories,
    single_articulated_trajectory,
    single_bicycle_trajectory,
    single_differential_trajectory,
    trajectories_to_dataframe,
)
from polymath_kinematics.explorer.config import (
    DEFAULT_ARTICULATED_FRONT_OVERHANG_M,
    DEFAULT_ARTICULATED_REAR_OVERHANG_M,
    DEFAULT_BICYCLE_FRONT_OVERHANG_M,
    DEFAULT_BICYCLE_REAR_OVERHANG_M,
    DEFAULT_DIFFERENTIAL_FRONT_OVERHANG_M,
    DEFAULT_DIFFERENTIAL_REAR_OVERHANG_M,
)


@st.cache_data
def _cached_lattice_differential(
    wheel_radius: float,
    track_width: float,
    base_wheel_velocities: tuple[float, ...],
    wheel_velocity_diffs: tuple[float, ...],
    duration: float,
    time_step: float,
    front_overhang: float,
    rear_overhang: float,
) -> list[DifferentialTrajectory]:
    return generate_lattice_differential(
        wheel_radius,
        track_width,
        base_wheel_velocities,
        wheel_velocity_diffs,
        duration,
        time_step,
        # Pose reference is the body centre, so the overhangs are the bumper distances directly.
        front_overhang_m=front_overhang,
        rear_overhang_m=rear_overhang,
        body_width_m=track_width,
    )


@st.cache_data
def _cached_lattice_bicycle(
    wheelbase: float,
    track_width: float,
    wheel_radius: float,
    drive_velocities: tuple[float, ...],
    steering_angles: tuple[float, ...],
    duration: float,
    time_step: float,
    front_overhang: float,
    rear_overhang: float,
) -> list[BicycleTrajectory]:
    return generate_lattice_bicycle(
        wheelbase,
        track_width,
        wheel_radius,
        drive_velocities,
        steering_angles,
        duration,
        time_step,
        # Pose reference is the rear axle; front bumper = wheelbase + overhang.
        front_overhang_m=wheelbase + front_overhang,
        rear_overhang_m=rear_overhang,
        body_width_m=track_width,
    )


@st.cache_data
def _cached_lattice_articulated(
    articulation_to_front: float,
    articulation_to_rear: float,
    front_track: float,
    rear_track: float,
    front_wheel_radius: float,
    rear_wheel_radius: float,
    drive_velocities: tuple[float, ...],
    articulation_angles: tuple[float, ...],
    duration: float,
    time_step: float,
    front_overhang: float,
    rear_overhang: float,
) -> list[ArticulatedTrajectory]:
    return generate_lattice_articulated(
        articulation_to_front,
        articulation_to_rear,
        front_track,
        rear_track,
        front_wheel_radius,
        rear_wheel_radius,
        drive_velocities,
        articulation_angles,
        duration,
        time_step,
        # base_link is the articulation joint; joint-to-bumper = joint-to-axle + overhang.
        front_joint_to_bumper_m=articulation_to_front + front_overhang,
        front_body_width_m=front_track,
        rear_joint_to_bumper_m=articulation_to_rear + rear_overhang,
        rear_body_width_m=rear_track,
    )


@st.cache_data
def _cached_single_bicycle(
    wheelbase: float,
    track_width: float,
    wheel_radius: float,
    initial_steering_angle_rad: float,
    target_steering_angle_rad: float,
    steering_rate_rad_s: float,
    drive_velocity: float,
    duration: float,
    time_step: float,
    front_overhang: float,
    rear_overhang: float,
) -> BicycleTrajectory:
    return single_bicycle_trajectory(
        wheelbase,
        track_width,
        wheel_radius,
        initial_steering_angle_rad=initial_steering_angle_rad,
        target_steering_angle_rad=target_steering_angle_rad,
        steering_rate_rad_s=steering_rate_rad_s,
        drive_velocity=drive_velocity,
        duration=duration,
        time_step=time_step,
        # Pose reference is the rear axle; see _cached_lattice_bicycle.
        front_overhang_m=wheelbase + front_overhang,
        rear_overhang_m=rear_overhang,
        body_width_m=track_width,
    )


@st.cache_data
def _cached_single_articulated(
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
    time_step: float,
    front_overhang: float,
    rear_overhang: float,
) -> ArticulatedTrajectory:
    return single_articulated_trajectory(
        articulation_to_front,
        articulation_to_rear,
        front_track,
        rear_track,
        front_wheel_radius,
        rear_wheel_radius,
        initial_articulation_angle_rad=initial_articulation_angle_rad,
        target_articulation_angle_rad=target_articulation_angle_rad,
        articulation_rate_rad_s=articulation_rate_rad_s,
        drive_velocity=drive_velocity,
        duration=duration,
        time_step=time_step,
        # base_link is the articulation joint; joint-to-bumper = joint-to-axle + overhang.
        front_joint_to_bumper_m=articulation_to_front + front_overhang,
        front_body_width_m=front_track,
        rear_joint_to_bumper_m=articulation_to_rear + rear_overhang,
        rear_body_width_m=rear_track,
    )


@st.cache_data
def _cached_single_differential(
    wheel_radius: float,
    track_width: float,
    initial_linear_velocity: float,
    initial_angular_velocity: float,
    target_linear_velocity: float,
    target_angular_velocity: float,
    linear_acceleration: float,
    angular_acceleration: float,
    duration: float,
    time_step: float,
    front_overhang: float,
    rear_overhang: float,
) -> DifferentialTrajectory:
    return single_differential_trajectory(
        wheel_radius,
        track_width,
        initial_linear_velocity=initial_linear_velocity,
        initial_angular_velocity=initial_angular_velocity,
        target_linear_velocity=target_linear_velocity,
        target_angular_velocity=target_angular_velocity,
        linear_acceleration=linear_acceleration,
        angular_acceleration=angular_acceleration,
        duration=duration,
        time_step=time_step,
        # Pose reference is the body centre; see _cached_lattice_differential.
        front_overhang_m=front_overhang,
        rear_overhang_m=rear_overhang,
        body_width_m=track_width,
    )


def get_config_dict(model_type: str) -> dict:
    """Get current configuration as a dictionary (depends on session state)."""
    config = {
        'model_type': model_type,
        'timestamp': datetime.now().isoformat(),
        'simulation': {
            'duration': st.session_state.sim_duration,
            'dt': st.session_state.sim_dt,
        },
    }

    if model_type == 'Differential Drive':
        config['model_parameters'] = {
            'wheel_radius': st.session_state.diff_wheel_radius,
            'track_width': st.session_state.diff_track_width,
            'front_overhang': st.session_state.diff_front_overhang,
            'rear_overhang': st.session_state.diff_rear_overhang,
        }
        config['control_inputs'] = {
            'base_vel_min': st.session_state.diff_base_vel_min,
            'base_vel_max': st.session_state.diff_base_vel_max,
            'n_base_vels': st.session_state.diff_n_base_vels,
            'max_wheel_diff': st.session_state.diff_max_wheel_diff,
            'n_samples': st.session_state.diff_n_samples,
        }
    elif model_type == 'Bicycle':
        config['model_parameters'] = {
            'wheelbase': st.session_state.bike_wheelbase,
            'track_width': st.session_state.bike_track_width,
            'wheel_radius': st.session_state.bike_wheel_radius,
            'front_overhang': st.session_state.bike_front_overhang,
            'rear_overhang': st.session_state.bike_rear_overhang,
        }
        config['control_inputs'] = {
            'v_min': st.session_state.bike_v_min,
            'v_max': st.session_state.bike_v_max,
            'n_velocities': st.session_state.bike_n_velocities,
            'max_steer_deg': st.session_state.bike_max_steer,
            'n_steer': st.session_state.bike_n_steer,
        }
    elif model_type == 'Articulated':
        config['model_parameters'] = {
            'articulation_to_front_axle': st.session_state.art_to_front,
            'articulation_to_rear_axle': st.session_state.art_to_rear,
            'front_track_width': st.session_state.art_front_track,
            'rear_track_width': st.session_state.art_rear_track,
            'front_wheel_radius': st.session_state.art_front_wheel_r,
            'rear_wheel_radius': st.session_state.art_rear_wheel_r,
            'front_overhang': st.session_state.art_front_overhang,
            'rear_overhang': st.session_state.art_rear_overhang,
        }
        config['control_inputs'] = {
            'v_min': st.session_state.art_v_min,
            'v_max': st.session_state.art_v_max,
            'n_velocities': st.session_state.art_n_velocities,
            'max_articulation_deg': st.session_state.art_max_angle,
            'n_angles': st.session_state.art_n_angles,
        }

    return config


def init_session_state():
    """Initialize default values for all model parameters."""
    defaults = {
        # Differential Drive
        'diff_wheel_radius': 0.1,
        'diff_track_width': 0.5,
        'diff_front_overhang': DEFAULT_DIFFERENTIAL_FRONT_OVERHANG_M,
        'diff_rear_overhang': DEFAULT_DIFFERENTIAL_REAR_OVERHANG_M,
        'diff_base_vel_min': 5.0,
        'diff_base_vel_max': 15.0,
        'diff_n_base_vels': 3,
        'diff_max_wheel_diff': 10.0,
        'diff_n_samples': 9,
        # Bicycle
        'bike_wheelbase': 2.5,
        'bike_track_width': 1.5,
        'bike_wheel_radius': 0.3,
        'bike_front_overhang': DEFAULT_BICYCLE_FRONT_OVERHANG_M,
        'bike_rear_overhang': DEFAULT_BICYCLE_REAR_OVERHANG_M,
        'bike_v_min': 1.0,
        'bike_v_max': 3.0,
        'bike_n_velocities': 3,
        'bike_max_steer': 30.0,
        'bike_n_steer': 9,
        # Articulated
        'art_to_front': 1.5,
        'art_to_rear': 1.2,
        'art_front_track': 1.8,
        'art_rear_track': 1.6,
        'art_front_overhang': DEFAULT_ARTICULATED_FRONT_OVERHANG_M,
        'art_rear_overhang': DEFAULT_ARTICULATED_REAR_OVERHANG_M,
        'art_front_wheel_r': 0.4,
        'art_rear_wheel_r': 0.5,
        'art_v_min': 1.0,
        'art_v_max': 3.0,
        'art_n_velocities': 3,
        'art_max_angle': 30.0,
        'art_n_angles': 9,
        # Simulation
        'sim_duration': 3.0,
        'sim_dt': 0.02,
        'export_dpi': 150,
        'export_format': 'png',
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value


st.set_page_config(page_title='Kinematic Explorer', layout='wide')
st.title('Kinematic Model Explorer')

init_session_state()

# Sidebar - Model Selection
st.sidebar.header('Model Configuration')
model_type = st.sidebar.selectbox('Model Type', ['Differential Drive', 'Bicycle', 'Articulated'])

st.sidebar.subheader('Model Parameters')

# Model-specific parameters
if model_type == 'Differential Drive':
    st.session_state.diff_wheel_radius = st.sidebar.slider(
        'Wheel Radius (m)', 0.05, 0.5, st.session_state.diff_wheel_radius, 0.01
    )
    st.session_state.diff_track_width = st.sidebar.slider(
        'Track Width (m)', 0.2, 2.0, st.session_state.diff_track_width, 0.05
    )
    st.session_state.diff_front_overhang = st.sidebar.slider(
        'Front Overhang (m)',
        0.0,
        2.0,
        st.session_state.diff_front_overhang,
        0.05,
        help='Body length ahead of the body centre (the pose reference).',
    )
    st.session_state.diff_rear_overhang = st.sidebar.slider(
        'Rear Overhang (m)',
        0.0,
        2.0,
        st.session_state.diff_rear_overhang,
        0.05,
        help='Body length behind the body centre (the pose reference).',
    )
    st.sidebar.markdown('---')
    st.sidebar.markdown(f'**Wheel Radius:** {st.session_state.diff_wheel_radius:.2f} m')
    st.sidebar.markdown(f'**Track Width:** {st.session_state.diff_track_width:.2f} m')
    st.sidebar.markdown(
        f'**Body Length:** {st.session_state.diff_front_overhang + st.session_state.diff_rear_overhang:.2f} m'
    )

elif model_type == 'Bicycle':
    st.session_state.bike_wheelbase = st.sidebar.slider('Wheelbase (m)', 1.0, 5.0, st.session_state.bike_wheelbase, 0.1)
    st.session_state.bike_track_width = st.sidebar.slider(
        'Track Width (m)', 1.0, 3.0, st.session_state.bike_track_width, 0.1
    )
    st.session_state.bike_wheel_radius = st.sidebar.slider(
        'Wheel Radius (m)', 0.1, 0.6, st.session_state.bike_wheel_radius, 0.05
    )
    st.session_state.bike_front_overhang = st.sidebar.slider(
        'Front Overhang (m)',
        0.0,
        2.0,
        st.session_state.bike_front_overhang,
        0.05,
        help='Body length ahead of the FRONT axle. The pose reference is the rear axle, so the '
        'front bumper sits a wheelbase plus this overhang ahead of it.',
    )
    st.session_state.bike_rear_overhang = st.sidebar.slider(
        'Rear Overhang (m)',
        0.0,
        2.0,
        st.session_state.bike_rear_overhang,
        0.05,
        help='Body length behind the rear axle (the pose reference).',
    )
    st.sidebar.markdown('---')
    st.sidebar.markdown(f'**Wheelbase:** {st.session_state.bike_wheelbase:.2f} m')
    st.sidebar.markdown(f'**Track Width:** {st.session_state.bike_track_width:.2f} m')
    st.sidebar.markdown(
        f'**Body Length:** '
        f'{st.session_state.bike_wheelbase + st.session_state.bike_front_overhang + st.session_state.bike_rear_overhang:.2f} m'
    )

elif model_type == 'Articulated':
    st.session_state.art_to_front = st.sidebar.slider(
        'Articulation to Front Axle (m)', 0.5, 3.0, st.session_state.art_to_front, 0.1
    )
    st.session_state.art_to_rear = st.sidebar.slider(
        'Articulation to Rear Axle (m)', 0.5, 3.0, st.session_state.art_to_rear, 0.1
    )
    st.session_state.art_front_track = st.sidebar.slider(
        'Front Track Width (m)', 1.0, 3.0, st.session_state.art_front_track, 0.1
    )
    st.session_state.art_rear_track = st.sidebar.slider(
        'Rear Track Width (m)', 1.0, 3.0, st.session_state.art_rear_track, 0.1
    )
    st.session_state.art_front_overhang = st.sidebar.slider(
        'Front Overhang (m)',
        0.0,
        3.0,
        st.session_state.art_front_overhang,
        0.1,
        help='Body length ahead of the front axle (e.g. bucket).',
    )
    st.session_state.art_rear_overhang = st.sidebar.slider(
        'Rear Overhang (m)',
        0.0,
        3.0,
        st.session_state.art_rear_overhang,
        0.1,
        help='Body length behind the rear axle (e.g. counterweight).',
    )
    st.session_state.art_front_wheel_r = st.sidebar.slider(
        'Front Wheel Radius (m)', 0.2, 0.8, st.session_state.art_front_wheel_r, 0.05
    )
    st.session_state.art_rear_wheel_r = st.sidebar.slider(
        'Rear Wheel Radius (m)', 0.2, 0.8, st.session_state.art_rear_wheel_r, 0.05
    )
    st.sidebar.markdown('---')
    st.sidebar.markdown(f'**Total Wheelbase:** {st.session_state.art_to_front + st.session_state.art_to_rear:.2f} m')

# Lattice Configuration
st.sidebar.header('Lattice Configuration')
st.session_state.sim_duration = st.sidebar.slider(
    'Simulation Duration (s)', 1.0, 10.0, st.session_state.sim_duration, 0.5
)

with st.sidebar.expander('Advanced Settings'):
    st.caption('Trajectories come from the C++ projectors (Euler integration at the time step below).')
    st.session_state.sim_dt = st.slider(
        'Time Step (s)',
        0.005,
        0.1,
        st.session_state.sim_dt,
        0.005,
        help='Smaller values are more accurate but slower',
    )
    st.caption(f'Steps per trajectory: {int(st.session_state.sim_duration / st.session_state.sim_dt)}')

# Generate trajectories based on model type
trajectories: list[AnyTrajectory]
group_values: list[float]

if model_type == 'Differential Drive':
    st.sidebar.subheader('Control Inputs')
    col1, col2 = st.sidebar.columns(2)
    with col1:
        st.session_state.diff_base_vel_min = st.number_input(
            'Min Base Vel (rad/s)', 1.0, 20.0, st.session_state.diff_base_vel_min, 1.0
        )
    with col2:
        st.session_state.diff_base_vel_max = st.number_input(
            'Max Base Vel (rad/s)', 1.0, 30.0, st.session_state.diff_base_vel_max, 1.0
        )
    st.session_state.diff_n_base_vels = st.sidebar.slider(
        'Number of Base Velocities', 1, 5, st.session_state.diff_n_base_vels
    )
    base_wheel_velocities = tuple(
        np.linspace(
            st.session_state.diff_base_vel_min, st.session_state.diff_base_vel_max, st.session_state.diff_n_base_vels
        )
    )

    st.session_state.diff_max_wheel_diff = st.sidebar.slider(
        'Max Wheel Velocity Diff (rad/s)', 1.0, 20.0, st.session_state.diff_max_wheel_diff, 1.0
    )
    st.session_state.diff_n_samples = st.sidebar.slider('Diff Samples', 3, 15, st.session_state.diff_n_samples, 2)
    wheel_diffs = tuple(
        np.linspace(
            -st.session_state.diff_max_wheel_diff, st.session_state.diff_max_wheel_diff, st.session_state.diff_n_samples
        )
    )

    trajectories = _cached_lattice_differential(
        wheel_radius=st.session_state.diff_wheel_radius,
        track_width=st.session_state.diff_track_width,
        base_wheel_velocities=base_wheel_velocities,
        wheel_velocity_diffs=wheel_diffs,
        duration=st.session_state.sim_duration,
        time_step=st.session_state.sim_dt,
        front_overhang=st.session_state.diff_front_overhang,
        rear_overhang=st.session_state.diff_rear_overhang,
    )
    group_values = list(base_wheel_velocities)

elif model_type == 'Bicycle':
    st.sidebar.subheader('Control Inputs')
    col1, col2 = st.sidebar.columns(2)
    with col1:
        st.session_state.bike_v_min = st.number_input('Min Drive Vel (m/s)', 0.5, 5.0, st.session_state.bike_v_min, 0.5)
    with col2:
        st.session_state.bike_v_max = st.number_input(
            'Max Drive Vel (m/s)', 0.5, 10.0, st.session_state.bike_v_max, 0.5
        )
    st.session_state.bike_n_velocities = st.sidebar.slider(
        'Number of Velocities', 1, 5, st.session_state.bike_n_velocities
    )
    drive_velocities = tuple(
        np.linspace(st.session_state.bike_v_min, st.session_state.bike_v_max, st.session_state.bike_n_velocities)
    )

    st.session_state.bike_max_steer = st.sidebar.slider(
        'Max Steering Angle (deg)', 5.0, 45.0, st.session_state.bike_max_steer, 5.0
    )
    st.session_state.bike_n_steer = st.sidebar.slider('Steering Samples', 3, 15, st.session_state.bike_n_steer, 2)
    steering_angles = tuple(
        np.linspace(
            -np.radians(st.session_state.bike_max_steer),
            np.radians(st.session_state.bike_max_steer),
            st.session_state.bike_n_steer,
        )
    )

    trajectories = _cached_lattice_bicycle(
        wheelbase=st.session_state.bike_wheelbase,
        track_width=st.session_state.bike_track_width,
        wheel_radius=st.session_state.bike_wheel_radius,
        drive_velocities=drive_velocities,
        steering_angles=steering_angles,
        duration=st.session_state.sim_duration,
        time_step=st.session_state.sim_dt,
        front_overhang=st.session_state.bike_front_overhang,
        rear_overhang=st.session_state.bike_rear_overhang,
    )
    group_values = list(drive_velocities)

else:  # Articulated
    st.sidebar.subheader('Control Inputs')
    col1, col2 = st.sidebar.columns(2)
    with col1:
        st.session_state.art_v_min = st.number_input('Min Drive Vel (m/s)', 0.5, 5.0, st.session_state.art_v_min, 0.5)
    with col2:
        st.session_state.art_v_max = st.number_input('Max Drive Vel (m/s)', 0.5, 10.0, st.session_state.art_v_max, 0.5)
    st.session_state.art_n_velocities = st.sidebar.slider(
        'Number of Velocities', 1, 5, st.session_state.art_n_velocities
    )
    drive_velocities = tuple(
        np.linspace(st.session_state.art_v_min, st.session_state.art_v_max, st.session_state.art_n_velocities)
    )

    st.session_state.art_max_angle = st.sidebar.slider(
        'Max Articulation Angle (deg)', 5.0, 45.0, st.session_state.art_max_angle, 5.0
    )
    st.session_state.art_n_angles = st.sidebar.slider('Articulation Samples', 3, 15, st.session_state.art_n_angles, 2)
    articulation_angles = tuple(
        np.linspace(
            -np.radians(st.session_state.art_max_angle),
            np.radians(st.session_state.art_max_angle),
            st.session_state.art_n_angles,
        )
    )

    trajectories = _cached_lattice_articulated(
        articulation_to_front=st.session_state.art_to_front,
        articulation_to_rear=st.session_state.art_to_rear,
        front_track=st.session_state.art_front_track,
        rear_track=st.session_state.art_rear_track,
        front_wheel_radius=st.session_state.art_front_wheel_r,
        rear_wheel_radius=st.session_state.art_rear_wheel_r,
        drive_velocities=drive_velocities,
        articulation_angles=articulation_angles,
        duration=st.session_state.sim_duration,
        time_step=st.session_state.sim_dt,
        front_overhang=st.session_state.art_front_overhang,
        rear_overhang=st.session_state.art_rear_overhang,
    )
    group_values = list(drive_velocities)

# Kinematic Equations Section
with st.expander('Kinematic Equations', expanded=False):
    eq_info = KINEMATIC_EQUATIONS[model_type]
    st.subheader(eq_info['title'])
    for eq in eq_info['equations']:
        st.latex(eq)
    st.markdown(eq_info['variables'])
    st.markdown('---')
    st.markdown(TRAJECTORY_EQUATIONS)

# Trajectory Lattice — only the longest (highest-velocity) group is shown. Lower velocities are
# the same trajectory fan at a shorter distance, so rendering all of them is redundant.
longest_group = [max(group_values)] if group_values else group_values
st.header('Trajectory Lattice')
lattice_fig = plot_lattice(trajectories, model_type, longest_group)
lattice_col, _ = st.columns([2, 1])  # constrain to 2/3 page width so the 16:9 plot isn't full-bleed
lattice_col.pyplot(lattice_fig)
plt.close(lattice_fig)

# Kinematic Analysis
st.header('Kinematic Analysis')
analysis_fig = plot_analysis(trajectories, model_type, group_values)
analysis_col, _ = st.columns([2, 1])  # constrain to 2/3 page width so the plot isn't full-bleed
analysis_col.pyplot(analysis_fig)
plt.close(analysis_fig)

# Vehicle Footprint Visualization
st.header('Vehicle Visualization')

vel_options = list(group_values)
# Body outlines come from the projector footprints; model_params only carries what the corners
# can't give us — the bicycle's front-axle offset and track width.
model_params: dict = {}
if model_type == 'Differential Drive':
    vel_label = 'Base Wheel Velocity (rad/s)'
    body_length = st.session_state.diff_front_overhang + st.session_state.diff_rear_overhang
    st.caption(f'Vehicle dimensions: {body_length:.2f}m x {st.session_state.diff_track_width:.2f}m')
elif model_type == 'Bicycle':
    model_params = {
        'wheelbase': st.session_state.bike_wheelbase,
        'track_width': st.session_state.bike_track_width,
    }
    vel_label = 'Drive Velocity (m/s)'
    body_length = (
        st.session_state.bike_wheelbase + st.session_state.bike_front_overhang + st.session_state.bike_rear_overhang
    )
    st.caption(
        f'Vehicle dimensions: {body_length:.2f}m x {st.session_state.bike_track_width:.2f}m '
        f'(wheelbase={st.session_state.bike_wheelbase:.2f}m)'
    )
else:  # Articulated
    vel_label = 'Drive Velocity (m/s)'
    # Body lengths are measured from the articulation joint (base_link) to each bumper:
    # joint-to-axle + overhang.
    front_length = st.session_state.art_to_front + st.session_state.art_front_overhang
    rear_length = st.session_state.art_to_rear + st.session_state.art_rear_overhang
    st.caption(
        f'Articulated: front={front_length:.2f}m x {st.session_state.art_front_track:.2f}m, '
        f'rear={rear_length:.2f}m x {st.session_state.art_rear_track:.2f}m'
    )

col1, col2 = st.columns([3, 1])
with col2:
    st.markdown('**Visualization Settings**')
    num_footprints = st.slider('Footprints per trajectory', 3, 8, 4, 1, key='num_footprints')

    if model_type == 'Differential Drive':
        angle_label = 'Angular velocities'
    elif model_type == 'Bicycle':
        angle_label = 'Steering angles'
    else:
        angle_label = 'Articulation angles'

    num_angles_viz = st.slider(
        angle_label, 1, 9, 5, 2, key='num_angles_viz', help='Number of angles to show (symmetric)'
    )

    if len(vel_options) > 1:
        viz_vel = st.select_slider(
            vel_label,
            options=[round(v, 1) for v in vel_options],
            value=round(vel_options[len(vel_options) // 2], 1),
            key='viz_velocity',
        )
    else:
        viz_vel = vel_options[0]
        st.caption(f'{vel_label}: {viz_vel:.1f}')

with col1:
    config = LATTICE_CONFIG[model_type]
    # Narrow to the chosen velocity first, then let the library pick a symmetric angle subset.
    velocity_filtered = [
        trajectory for trajectory in trajectories if abs(get_traj_attr(trajectory, config.vel_key) - viz_vel) < 0.05
    ]
    viz_trajectories = select_symmetric_trajectories(
        velocity_filtered, model_type, num_angles=num_angles_viz, num_velocities=1
    )

    if viz_trajectories:
        footprint_fig = plot_trajectory_with_footprints(viz_trajectories, model_type, model_params, num_footprints)
        st.pyplot(footprint_fig)
        plt.close(footprint_fig)
    else:
        st.warning('No trajectories selected. Adjust the visualization settings.')

# Single Projected Trajectory — ramps the model command from initial → target at a chosen rate,
# clamped to [min, max], and plots the resulting trajectory live. Velocity/horizon are inherited.
st.header('Single Projected Trajectory')
st.caption(
    'Set the initial input, target input, and rate of change. The plot updates as you move the '
    'sliders. Drive velocity and simulation horizon are inherited from the sections above.'
)


def _render_single_trajectory(single_traj):
    """Render one projected trajectory with footprints (shared by all model branches below)."""
    fig = plot_trajectory_with_footprints([single_traj], model_type, model_params, num_footprints=5)
    plot_col, _ = st.columns([2, 1])  # constrain to 2/3 page width so the 16:9 plot isn't full-bleed
    plot_col.pyplot(fig)
    plt.close(fig)


if model_type == 'Bicycle':
    col_a, col_b, col_c = st.columns(3)
    with col_a:
        single_initial_deg = st.slider(
            'Initial Steering (deg)',
            -st.session_state.bike_max_steer,
            st.session_state.bike_max_steer,
            0.0,
            1.0,
            key='bike_single_initial_deg',
        )
    with col_b:
        single_target_deg = st.slider(
            'Target Steering (deg)',
            -st.session_state.bike_max_steer,
            st.session_state.bike_max_steer,
            float(st.session_state.bike_max_steer / 2),
            1.0,
            key='bike_single_target_deg',
        )
    with col_c:
        single_rate_deg_s = st.slider(
            'Steering Rate (deg/s)',
            0.0,
            180.0,
            30.0,
            1.0,
            key='bike_single_rate_deg_s',
        )

    single_traj = _cached_single_bicycle(
        wheelbase=st.session_state.bike_wheelbase,
        track_width=st.session_state.bike_track_width,
        wheel_radius=st.session_state.bike_wheel_radius,
        initial_steering_angle_rad=float(np.radians(single_initial_deg)),
        target_steering_angle_rad=float(np.radians(single_target_deg)),
        steering_rate_rad_s=float(np.radians(single_rate_deg_s)),
        drive_velocity=float(viz_vel),
        duration=st.session_state.sim_duration,
        time_step=st.session_state.sim_dt,
        front_overhang=st.session_state.bike_front_overhang,
        rear_overhang=st.session_state.bike_rear_overhang,
    )
    _render_single_trajectory(single_traj)
    st.caption(
        f'Drive velocity: {viz_vel:.2f} m/s · '
        f'Horizon: {st.session_state.sim_duration:.1f}s · '
        f'dt: {st.session_state.sim_dt:.3f}s'
    )

elif model_type == 'Articulated':
    col_a, col_b, col_c = st.columns(3)
    with col_a:
        single_initial_deg = st.slider(
            'Initial Articulation (deg)',
            -st.session_state.art_max_angle,
            st.session_state.art_max_angle,
            0.0,
            1.0,
            key='art_single_initial_deg',
        )
    with col_b:
        single_target_deg = st.slider(
            'Target Articulation (deg)',
            -st.session_state.art_max_angle,
            st.session_state.art_max_angle,
            float(st.session_state.art_max_angle / 2),
            1.0,
            key='art_single_target_deg',
        )
    with col_c:
        single_rate_deg_s = st.slider(
            'Articulation Rate (deg/s)',
            0.0,
            90.0,
            15.0,
            1.0,
            key='art_single_rate_deg_s',
        )

    single_traj = _cached_single_articulated(
        articulation_to_front=st.session_state.art_to_front,
        articulation_to_rear=st.session_state.art_to_rear,
        front_track=st.session_state.art_front_track,
        rear_track=st.session_state.art_rear_track,
        front_wheel_radius=st.session_state.art_front_wheel_r,
        rear_wheel_radius=st.session_state.art_rear_wheel_r,
        initial_articulation_angle_rad=float(np.radians(single_initial_deg)),
        target_articulation_angle_rad=float(np.radians(single_target_deg)),
        articulation_rate_rad_s=float(np.radians(single_rate_deg_s)),
        drive_velocity=float(viz_vel),
        duration=st.session_state.sim_duration,
        time_step=st.session_state.sim_dt,
        front_overhang=st.session_state.art_front_overhang,
        rear_overhang=st.session_state.art_rear_overhang,
    )
    _render_single_trajectory(single_traj)
    st.caption(
        f'Drive velocity: {viz_vel:.2f} m/s · '
        f'Horizon: {st.session_state.sim_duration:.1f}s · '
        f'dt: {st.session_state.sim_dt:.3f}s'
    )

else:  # Differential Drive — ramp linear AND angular body command.
    st.markdown('**Body command**')
    col_a, col_b, col_c, col_d = st.columns(4)
    with col_a:
        single_initial_v = st.slider(
            'Initial Linear (m/s)',
            -3.0,
            3.0,
            0.0,
            0.1,
            key='diff_single_initial_v',
        )
    with col_b:
        single_target_v = st.slider(
            'Target Linear (m/s)',
            -3.0,
            3.0,
            1.0,
            0.1,
            key='diff_single_target_v',
        )
    with col_c:
        single_initial_omega = st.slider(
            'Initial Angular (rad/s)',
            -3.0,
            3.0,
            0.0,
            0.1,
            key='diff_single_initial_omega',
        )
    with col_d:
        single_target_omega = st.slider(
            'Target Angular (rad/s)',
            -3.0,
            3.0,
            0.5,
            0.1,
            key='diff_single_target_omega',
        )
    st.markdown('**Acceleration limits**')
    col_e, col_f = st.columns(2)
    with col_e:
        single_linear_accel = st.slider(
            'Linear Accel (m/s²)',
            0.0,
            5.0,
            1.0,
            0.1,
            key='diff_single_linear_accel',
        )
    with col_f:
        single_angular_accel = st.slider(
            'Angular Accel (rad/s²)',
            0.0,
            5.0,
            1.0,
            0.1,
            key='diff_single_angular_accel',
        )

    single_traj = _cached_single_differential(
        wheel_radius=st.session_state.diff_wheel_radius,
        track_width=st.session_state.diff_track_width,
        initial_linear_velocity=float(single_initial_v),
        initial_angular_velocity=float(single_initial_omega),
        target_linear_velocity=float(single_target_v),
        target_angular_velocity=float(single_target_omega),
        linear_acceleration=float(single_linear_accel),
        angular_acceleration=float(single_angular_accel),
        duration=st.session_state.sim_duration,
        time_step=st.session_state.sim_dt,
        front_overhang=st.session_state.diff_front_overhang,
        rear_overhang=st.session_state.diff_rear_overhang,
    )
    _render_single_trajectory(single_traj)
    st.caption(f'Horizon: {st.session_state.sim_duration:.1f}s · dt: {st.session_state.sim_dt:.3f}s')

# Parameter Table
with st.expander('Current Configuration', expanded=False):
    config_dict = get_config_dict(model_type)

    col1, col2 = st.columns(2)
    with col1:
        st.markdown('**Model Parameters:**')
        for key, value in config_dict['model_parameters'].items():
            st.markdown(f'- {key.replace("_", " ").title()}: `{value}`')

    with col2:
        st.markdown('**Control Inputs:**')
        for key, value in config_dict['control_inputs'].items():
            st.markdown(f'- {key.replace("_", " ").title()}: `{value}`')

    st.markdown('**Simulation Settings:**')
    st.markdown(f'- Duration: `{config_dict["simulation"]["duration"]}s`, dt: `{config_dict["simulation"]["dt"]}s`')

# Summary stats
st.header('Summary')
col1, col2, col3, col4 = st.columns(4)

with col1:
    st.metric('Model Type', model_type)
with col2:
    st.metric('Total Trajectories', len(trajectories))
with col3:
    if model_type == 'Articulated':
        max_art = max(abs(trajectory.articulation_angle) for trajectory in trajectories)  # type: ignore[union-attr]
        st.metric('Max Articulation', f'{np.degrees(max_art):.1f} deg')
    elif model_type == 'Bicycle':
        max_steer = max(abs(trajectory.steering_angle) for trajectory in trajectories)  # type: ignore[union-attr]
        st.metric('Max Steering', f'{np.degrees(max_steer):.1f} deg')
    elif model_type == 'Differential Drive':
        st.metric('Track Width', f'{st.session_state.diff_track_width:.2f} m')
with col4:
    max_omega = max(abs(trajectory.angular_velocity) for trajectory in trajectories)
    st.metric('Max Angular Vel', f'{max_omega:.2f} rad/s')

# Export Section
st.sidebar.header('Export')

with st.sidebar.expander('Export Settings'):
    st.session_state.export_format = st.selectbox(
        'Image Format',
        ['png', 'svg', 'pdf'],
        index=['png', 'svg', 'pdf'].index(st.session_state.export_format),
    )
    st.session_state.export_dpi = st.select_slider(
        'DPI (for PNG)',
        [72, 150, 300, 600],
        value=st.session_state.export_dpi,
    )

# CSV Export
df = trajectories_to_dataframe(trajectories, model_type)
csv_buffer = io.StringIO()
df.to_csv(csv_buffer, index=False)
st.sidebar.download_button(
    label='Download Trajectory CSV',
    data=csv_buffer.getvalue(),
    file_name=f'trajectories_{model_type.lower().replace(" ", "_")}.csv',
    mime='text/csv',
)

# JSON Config Export
config_export = get_config_dict(model_type)
st.sidebar.download_button(
    label='Download Config JSON',
    data=json.dumps(config_export, indent=2),
    file_name=f'config_{model_type.lower().replace(" ", "_")}.json',
    mime='application/json',
)

# Image Export — matches the displayed lattice (longest/highest-velocity group only), rendered
# into memory so the browser downloads it rather than the server writing a file.
image_format = st.session_state.export_format
image_buffer = io.BytesIO()
export_fig = plot_lattice(trajectories, model_type, longest_group)
save_kwargs = {'bbox_inches': 'tight', 'format': image_format}
if image_format == 'png':
    save_kwargs['dpi'] = st.session_state.export_dpi
export_fig.savefig(image_buffer, **save_kwargs)
plt.close(export_fig)
st.sidebar.download_button(
    label=f'Download Lattice Image ({image_format.upper()})',
    data=image_buffer.getvalue(),
    file_name=f'lattice_{model_type.lower().replace(" ", "_")}.{image_format}',
    mime={'png': 'image/png', 'svg': 'image/svg+xml', 'pdf': 'application/pdf'}[image_format],
)
