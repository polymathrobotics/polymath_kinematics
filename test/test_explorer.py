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
"""Tests for the kinematic explorer module."""

import math

import pytest

from polymath_kinematics.explorer import (
    KINEMATIC_EQUATIONS,
    LATTICE_CONFIG,
    generate_lattice_articulated,
    generate_lattice_bicycle,
    generate_lattice_differential,
    plot_analysis,
    plot_lattice,
    plot_trajectory_with_footprints,
    select_symmetric_trajectories,
    trajectories_to_dataframe,
)


class TestLatticeGeneration:
    def test_generate_lattice_differential(self):
        trajectories = generate_lattice_differential(
            wheel_radius=0.1,
            track_width=0.5,
            base_wheel_velocities=(10.0,),
            wheel_velocity_diffs=(-5.0, 0.0, 5.0),
            duration=1.0,
            time_step=0.1,
        )

        # Differential drive still uses manual Euler: num_steps = int(duration/time_step).
        assert len(trajectories) == 3
        assert len(trajectories[0].time) == 10
        assert trajectories[0].linear_velocity == pytest.approx(1.0)

    def test_generate_lattice_differential_multiple_velocities(self):
        trajectories = generate_lattice_differential(
            wheel_radius=0.1,
            track_width=0.5,
            base_wheel_velocities=(10.0, 15.0),
            wheel_velocity_diffs=(-5.0, 0.0, 5.0),
            duration=1.0,
            time_step=0.1,
        )
        assert len(trajectories) == 6

    def test_generate_lattice_bicycle(self):
        trajectories = generate_lattice_bicycle(
            wheelbase=2.5,
            track_width=1.5,
            wheel_radius=0.3,
            drive_velocities=(2.0,),
            steering_angles=(0.0, math.radians(15)),
            duration=1.0,
            time_step=0.1,
        )
        # Projector returns ceil(duration/dt) + 1 samples (initial state seeded as element 0).
        assert len(trajectories) == 2
        assert len(trajectories[0].time) == 11
        assert trajectories[0].drive_velocity == pytest.approx(2.0)

    def test_generate_lattice_articulated(self):
        trajectories = generate_lattice_articulated(
            articulation_to_front=1.5,
            articulation_to_rear=1.2,
            front_track=1.8,
            rear_track=1.6,
            front_wheel_radius=0.4,
            rear_wheel_radius=0.5,
            drive_velocities=(2.0,),
            articulation_angles=(0.0, math.radians(15)),
            duration=1.0,
            time_step=0.1,
        )
        assert len(trajectories) == 2
        assert len(trajectories[0].time) == 11
        assert trajectories[0].drive_velocity == pytest.approx(2.0)

    def test_method_param_accepted_for_compatibility(self):
        # method='rk4' is accepted but the projector path is Euler-only — the call must still succeed.
        trajectories = generate_lattice_bicycle(
            wheelbase=2.5,
            track_width=1.5,
            wheel_radius=0.3,
            drive_velocities=(2.0,),
            steering_angles=(math.radians(15),),
            duration=1.0,
            time_step=0.1,
            method='rk4',
        )
        assert len(trajectories) == 1


class TestDataframeExport:
    def test_differential_drive_export(self):
        trajectories = generate_lattice_differential(
            wheel_radius=0.1,
            track_width=0.5,
            base_wheel_velocities=(10.0,),
            wheel_velocity_diffs=(-5.0, 0.0, 5.0),
            duration=1.0,
            time_step=0.1,
        )
        dataframe = trajectories_to_dataframe(trajectories, 'Differential Drive')

        expected_columns = [
            'trajectory_id',
            'time',
            'x',
            'y',
            'theta',
            'linear_velocity',
            'angular_velocity',
            'left_wheel_velocity',
            'right_wheel_velocity',
            'base_wheel_velocity',
        ]
        assert list(dataframe.columns) == expected_columns
        assert len(dataframe) == 30  # 3 trajectories * 10 points each

    def test_bicycle_export(self):
        trajectories = generate_lattice_bicycle(
            wheelbase=2.5,
            track_width=1.5,
            wheel_radius=0.3,
            drive_velocities=(2.0,),
            steering_angles=(0.0,),
            duration=1.0,
            time_step=0.1,
        )
        dataframe = trajectories_to_dataframe(trajectories, 'Bicycle')

        assert 'drive_velocity' in dataframe.columns
        assert 'steering_angle' in dataframe.columns
        assert 'turning_radius' in dataframe.columns

    def test_articulated_export(self):
        trajectories = generate_lattice_articulated(
            articulation_to_front=1.5,
            articulation_to_rear=1.2,
            front_track=1.8,
            rear_track=1.6,
            front_wheel_radius=0.4,
            rear_wheel_radius=0.5,
            drive_velocities=(2.0,),
            articulation_angles=(0.0,),
            duration=1.0,
            time_step=0.1,
        )
        dataframe = trajectories_to_dataframe(trajectories, 'Articulated')

        assert 'drive_velocity' in dataframe.columns
        assert 'articulation_angle' in dataframe.columns
        assert 'turning_radius' in dataframe.columns

    def test_empty_trajectories(self):
        dataframe = trajectories_to_dataframe([], 'Bicycle')
        assert dataframe.empty


class TestTrajectorySelection:
    def test_select_symmetric_trajectories(self):
        trajectories = generate_lattice_differential(
            wheel_radius=0.1,
            track_width=0.5,
            base_wheel_velocities=(10.0, 15.0),
            wheel_velocity_diffs=(-5.0, -2.5, 0.0, 2.5, 5.0),
            duration=1.0,
            time_step=0.1,
        )
        selected = select_symmetric_trajectories(trajectories, 'Differential Drive', num_angles=3, num_velocities=1)
        assert len(selected) == 3

    def test_select_all_when_fewer_than_requested(self):
        trajectories = generate_lattice_bicycle(
            wheelbase=2.5,
            track_width=1.5,
            wheel_radius=0.3,
            drive_velocities=(2.0,),
            steering_angles=(0.0, math.radians(15)),
            duration=1.0,
            time_step=0.1,
        )
        selected = select_symmetric_trajectories(trajectories, 'Bicycle', num_angles=5, num_velocities=1)
        assert len(selected) == 2

    def test_empty_trajectories_selection(self):
        selected = select_symmetric_trajectories([], 'Bicycle', num_angles=5, num_velocities=1)
        assert selected == []


class TestPlotting:
    @pytest.fixture
    def differential_trajectories(self):
        return generate_lattice_differential(
            wheel_radius=0.1,
            track_width=0.5,
            base_wheel_velocities=(10.0, 15.0),
            wheel_velocity_diffs=(-5.0, 0.0, 5.0),
            duration=1.0,
            time_step=0.1,
        )

    @pytest.fixture
    def bicycle_trajectories(self):
        return generate_lattice_bicycle(
            wheelbase=2.5,
            track_width=1.5,
            wheel_radius=0.3,
            drive_velocities=(2.0, 3.0),
            steering_angles=(-math.radians(15), 0.0, math.radians(15)),
            duration=1.0,
            time_step=0.1,
        )

    @pytest.fixture
    def articulated_trajectories(self):
        return generate_lattice_articulated(
            articulation_to_front=1.5,
            articulation_to_rear=1.2,
            front_track=1.8,
            rear_track=1.6,
            front_wheel_radius=0.4,
            rear_wheel_radius=0.5,
            drive_velocities=(2.0,),
            articulation_angles=(-math.radians(15), 0.0, math.radians(15)),
            duration=1.0,
            time_step=0.1,
        )

    def test_plot_lattice_differential(self, differential_trajectories):
        assert plot_lattice(differential_trajectories, 'Differential Drive', [10.0, 15.0]) is not None

    def test_plot_lattice_bicycle(self, bicycle_trajectories):
        assert plot_lattice(bicycle_trajectories, 'Bicycle', [2.0, 3.0]) is not None

    def test_plot_lattice_articulated(self, articulated_trajectories):
        assert plot_lattice(articulated_trajectories, 'Articulated', [2.0]) is not None

    def test_plot_analysis_differential(self, differential_trajectories):
        assert plot_analysis(differential_trajectories, 'Differential Drive', [10.0, 15.0]) is not None

    def test_plot_analysis_bicycle(self, bicycle_trajectories):
        assert plot_analysis(bicycle_trajectories, 'Bicycle', [2.0, 3.0]) is not None

    def test_plot_analysis_articulated(self, articulated_trajectories):
        assert plot_analysis(articulated_trajectories, 'Articulated', [2.0]) is not None

    def test_plot_trajectory_with_footprints_differential(self, differential_trajectories):
        selected = select_symmetric_trajectories(
            differential_trajectories, 'Differential Drive', num_angles=3, num_velocities=1
        )
        figure = plot_trajectory_with_footprints(
            selected, 'Differential Drive', {'length': 0.75, 'width': 0.5}, num_footprints=3
        )
        assert figure is not None

    def test_plot_trajectory_with_footprints_bicycle(self, bicycle_trajectories):
        selected = select_symmetric_trajectories(bicycle_trajectories, 'Bicycle', num_angles=3, num_velocities=1)
        figure = plot_trajectory_with_footprints(selected, 'Bicycle', {'length': 2.5, 'width': 1.5}, num_footprints=3)
        assert figure is not None

    def test_plot_trajectory_with_footprints_articulated(self, articulated_trajectories):
        selected = select_symmetric_trajectories(
            articulated_trajectories, 'Articulated', num_angles=3, num_velocities=1
        )
        model_params = {
            'front_length': 1.5,
            'rear_length': 1.2,
            'front_width': 1.8,
            'rear_width': 1.6,
        }
        figure = plot_trajectory_with_footprints(selected, 'Articulated', model_params, num_footprints=3)
        assert figure is not None


class TestConfig:
    def test_lattice_config_keys(self):
        assert 'Differential Drive' in LATTICE_CONFIG
        assert 'Bicycle' in LATTICE_CONFIG
        assert 'Articulated' in LATTICE_CONFIG

    def test_lattice_config_fields(self):
        config = LATTICE_CONFIG['Bicycle']
        assert config.group_key == 'drive_velocity'
        assert config.vel_key == 'drive_velocity'
        assert config.angle_key == 'steering_angle'

    def test_kinematic_equations_present(self):
        for model_type in ('Differential Drive', 'Bicycle', 'Articulated'):
            assert model_type in KINEMATIC_EQUATIONS
            assert 'title' in KINEMATIC_EQUATIONS[model_type]
            assert 'equations' in KINEMATIC_EQUATIONS[model_type]
            assert 'variables' in KINEMATIC_EQUATIONS[model_type]
