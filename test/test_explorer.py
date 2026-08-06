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

from polymath_kinematics import DifferentialDriveModel
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
    single_articulated_trajectory,
    single_bicycle_trajectory,
    single_differential_trajectory,
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

        # ceil(duration/dt) + 1 samples, same as the bicycle and articulated lattices.
        assert len(trajectories) == 3
        assert len(trajectories[0].time) == 11
        assert trajectories[0].linear_velocity == pytest.approx(1.0)

    def test_generate_lattice_differential_footprints(self):
        # With body dimensions supplied, the projector emits a footprint per sample.
        trajectories = generate_lattice_differential(
            wheel_radius=0.1,
            track_width=0.5,
            base_wheel_velocities=(10.0,),
            wheel_velocity_diffs=(0.0, 5.0),
            duration=1.0,
            time_step=0.1,
            front_overhang_m=0.4,
            rear_overhang_m=0.4,
            body_width_m=0.5,
        )
        for trajectory in trajectories:
            assert trajectory.footprint_series is not None
            assert trajectory.footprint_series.shape == (len(trajectory.time), 4, 2)

    def test_generate_lattice_differential_no_dims_has_no_footprints(self):
        trajectories = generate_lattice_differential(
            wheel_radius=0.1,
            track_width=0.5,
            base_wheel_velocities=(10.0,),
            wheel_velocity_diffs=(0.0,),
            duration=1.0,
            time_step=0.1,
        )
        assert trajectories[0].footprint_series is None

    def test_generate_lattice_differential_outer_cells_are_not_clamped(self):
        # Bounds must span the sweep, or the extreme cells get clamped back toward the middle.
        diffs = (-10.0, 0.0, 10.0)
        trajectories = generate_lattice_differential(
            wheel_radius=0.1,
            track_width=0.5,
            base_wheel_velocities=(10.0,),
            wheel_velocity_diffs=diffs,
            duration=1.0,
            time_step=0.1,
        )
        model = DifferentialDriveModel(0.1, 0.5)
        for trajectory, velocity_diff in zip(trajectories, diffs):
            expected = model.wheel_velocities_to_body_velocity(10.0 - velocity_diff / 2, 10.0 + velocity_diff / 2)
            assert trajectory.angular_velocity == pytest.approx(expected.angular_velocity_rad_s)
            assert trajectory.linear_velocity == pytest.approx(expected.linear_velocity_m_s)

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
        assert len(dataframe) == 33  # 3 trajectories * 11 samples each

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
        # Body dims are supplied so the footprint-overlay path is actually exercised.
        return generate_lattice_differential(
            wheel_radius=0.1,
            track_width=0.5,
            base_wheel_velocities=(10.0, 15.0),
            wheel_velocity_diffs=(-5.0, 0.0, 5.0),
            duration=1.0,
            time_step=0.1,
            front_overhang_m=0.4,
            rear_overhang_m=0.4,
            body_width_m=0.5,
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
            front_overhang_m=2.5 + 0.6,
            rear_overhang_m=0.5,
            body_width_m=1.5,
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
            front_joint_to_bumper_m=1.5 + 1.0,
            front_body_width_m=1.8,
            rear_joint_to_bumper_m=1.2 + 0.8,
            rear_body_width_m=1.6,
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
        figure = plot_trajectory_with_footprints(selected, 'Differential Drive', {}, num_footprints=3)
        assert figure is not None

    def test_plot_trajectory_with_footprints_bicycle(self, bicycle_trajectories):
        selected = select_symmetric_trajectories(bicycle_trajectories, 'Bicycle', num_angles=3, num_velocities=1)
        figure = plot_trajectory_with_footprints(
            selected, 'Bicycle', {'wheelbase': 2.5, 'track_width': 1.5}, num_footprints=3
        )
        assert figure is not None

    def test_plot_trajectory_with_footprints_articulated(self, articulated_trajectories):
        selected = select_symmetric_trajectories(
            articulated_trajectories, 'Articulated', num_angles=3, num_velocities=1
        )
        figure = plot_trajectory_with_footprints(selected, 'Articulated', {}, num_footprints=3)
        assert figure is not None

    def test_plot_trajectory_with_footprints_skips_missing_footprints(self):
        # No body dims: the path is still drawn, the overlay skipped rather than raising.
        without_dims = generate_lattice_bicycle(
            wheelbase=2.5,
            track_width=1.5,
            wheel_radius=0.3,
            drive_velocities=(2.0,),
            steering_angles=(math.radians(15),),
            duration=1.0,
            time_step=0.1,
        )
        assert without_dims[0].footprint_series is None
        figure = plot_trajectory_with_footprints(without_dims, 'Bicycle', {}, num_footprints=3)
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


class TestSingleTrajectory:
    def test_single_bicycle_trajectory_reaches_target(self):
        # Initial=0, target=0.3, rate=0.3 rad/s, duration=2s → step adds at most 0.3*dt;
        # over 2s we reach the target well before the horizon ends. The trajectory's
        # `steering_angle` field reflects the target (steady-state).
        traj = single_bicycle_trajectory(
            wheelbase=2.5,
            track_width=1.5,
            wheel_radius=0.3,
            initial_steering_angle_rad=0.0,
            target_steering_angle_rad=0.3,
            steering_rate_rad_s=0.3,
            drive_velocity=1.0,
            duration=2.0,
            time_step=0.1,
        )
        assert traj.steering_angle == pytest.approx(0.3)
        assert len(traj.time) == 21
        # Pose advanced from origin (we drove forward).
        assert traj.x[-1] > 0.0

    def test_single_articulated_trajectory_reaches_target(self):
        traj = single_articulated_trajectory(
            articulation_to_front=1.66,
            articulation_to_rear=1.44,
            front_track=2.0,
            rear_track=2.0,
            front_wheel_radius=0.723,
            rear_wheel_radius=0.723,
            initial_articulation_angle_rad=0.0,
            target_articulation_angle_rad=0.4,
            articulation_rate_rad_s=0.5,
            drive_velocity=1.0,
            duration=2.0,
            time_step=0.1,
        )
        assert traj.articulation_angle == pytest.approx(0.4)
        assert len(traj.time) == 21
        assert traj.x[-1] > 0.0

    def test_single_differential_trajectory_ramps_both_velocities(self):
        traj = single_differential_trajectory(
            wheel_radius=0.1,
            track_width=0.5,
            initial_linear_velocity=0.0,
            initial_angular_velocity=0.0,
            target_linear_velocity=1.0,
            target_angular_velocity=0.5,
            linear_acceleration=1.0,
            angular_acceleration=1.0,
            duration=2.0,
            time_step=0.1,
        )
        # Targets reached in ~1.0s and ~0.5s respectively; final state pinned to target.
        assert traj.linear_velocity == pytest.approx(1.0)
        assert traj.angular_velocity == pytest.approx(0.5)
        assert len(traj.time) == 21

    def test_single_bicycle_zero_rate_keeps_initial_angle(self):
        # With rate=0 the angle never advances; turning_radius reflects the (unchanging)
        # initial angle, not the target.
        traj = single_bicycle_trajectory(
            wheelbase=2.5,
            track_width=1.5,
            wheel_radius=0.3,
            initial_steering_angle_rad=0.0,
            target_steering_angle_rad=0.5,
            steering_rate_rad_s=0.0,
            drive_velocity=1.0,
            duration=1.0,
            time_step=0.1,
        )
        # Zero steering → straight line along +x.
        assert traj.x[-1] == pytest.approx(1.0)
        assert traj.y[-1] == pytest.approx(0.0)

    def test_single_bicycle_steering_series_brackets_initial_and_target(self):
        # Initial=0 → target=0.3 at rate=0.3 rad/s; ramp completes in 1s. Over duration=2s
        # the series starts at 0 and ends pinned at 0.3.
        traj = single_bicycle_trajectory(
            wheelbase=2.5,
            track_width=1.5,
            wheel_radius=0.3,
            initial_steering_angle_rad=0.0,
            target_steering_angle_rad=0.3,
            steering_rate_rad_s=0.3,
            drive_velocity=1.0,
            duration=2.0,
            time_step=0.1,
        )
        assert traj.steering_angle_series is not None
        assert len(traj.steering_angle_series) == len(traj.time)
        assert traj.steering_angle_series[0] == pytest.approx(0.0)
        assert traj.steering_angle_series[-1] == pytest.approx(0.3)

    def test_single_articulated_articulation_series_brackets_initial_and_target(self):
        traj = single_articulated_trajectory(
            articulation_to_front=1.66,
            articulation_to_rear=1.44,
            front_track=2.0,
            rear_track=2.0,
            front_wheel_radius=0.723,
            rear_wheel_radius=0.723,
            initial_articulation_angle_rad=0.0,
            target_articulation_angle_rad=0.4,
            articulation_rate_rad_s=0.5,
            drive_velocity=1.0,
            duration=2.0,
            time_step=0.1,
        )
        assert traj.articulation_angle_series is not None
        assert len(traj.articulation_angle_series) == len(traj.time)
        assert traj.articulation_angle_series[0] == pytest.approx(0.0)
        assert traj.articulation_angle_series[-1] == pytest.approx(0.4)

    def test_single_bicycle_footprint_series_none_without_dims(self):
        traj = single_bicycle_trajectory(
            wheelbase=2.5,
            track_width=1.5,
            wheel_radius=0.3,
            initial_steering_angle_rad=0.0,
            target_steering_angle_rad=0.2,
            steering_rate_rad_s=0.0,
            drive_velocity=1.0,
            duration=1.0,
            time_step=0.1,
        )
        # No footprint dimensions passed → projector emits empty footprints → series is None.
        assert traj.footprint_series is None

    def test_single_bicycle_footprint_series_present_with_dims(self):
        traj = single_bicycle_trajectory(
            wheelbase=2.5,
            track_width=1.5,
            wheel_radius=0.3,
            initial_steering_angle_rad=0.0,
            target_steering_angle_rad=0.2,
            steering_rate_rad_s=0.0,
            drive_velocity=1.0,
            duration=1.0,
            time_step=0.1,
            front_overhang_m=3.0,
            rear_overhang_m=1.0,
            body_width_m=1.5,
        )
        assert traj.footprint_series is not None
        # (N samples, 4 corners, xy)
        assert traj.footprint_series.shape == (len(traj.time), 4, 2)

    def test_single_articulated_footprint_series_present_with_dims(self):
        traj = single_articulated_trajectory(
            articulation_to_front=1.66,
            articulation_to_rear=1.44,
            front_track=2.0,
            rear_track=2.0,
            front_wheel_radius=0.723,
            rear_wheel_radius=0.723,
            initial_articulation_angle_rad=0.0,
            target_articulation_angle_rad=0.4,
            articulation_rate_rad_s=0.5,
            drive_velocity=1.0,
            duration=1.0,
            time_step=0.1,
            front_joint_to_bumper_m=2.2,
            front_body_width_m=2.0,
            rear_joint_to_bumper_m=2.0,
            rear_body_width_m=2.0,
        )
        assert traj.front_footprint_series is not None
        assert traj.rear_footprint_series is not None
        assert traj.front_footprint_series.shape == (len(traj.time), 4, 2)
        assert traj.rear_footprint_series.shape == (len(traj.time), 4, 2)

    def test_single_articulated_footprint_extends_behind_rear_axle(self):
        # base_link is the articulation joint. Straight ahead (gamma=0) from the origin along +x,
        # with rear_joint_to_bumper = articulation_to_rear + rear_overhang, the rear body's
        # rearmost point is at -(articulation_to_rear + rear_overhang) — i.e. behind the rear
        # axle (which sits at -articulation_to_rear).
        articulation_to_rear = 1.44
        rear_overhang = 0.8
        traj = single_articulated_trajectory(
            articulation_to_front=1.66,
            articulation_to_rear=articulation_to_rear,
            front_track=2.0,
            rear_track=2.0,
            front_wheel_radius=0.723,
            rear_wheel_radius=0.723,
            initial_articulation_angle_rad=0.0,
            target_articulation_angle_rad=0.0,
            articulation_rate_rad_s=0.0,
            drive_velocity=0.0,  # stay at the origin so the geometry is exact
            duration=0.2,
            time_step=0.1,
            front_joint_to_bumper_m=1.66 + 1.0,
            front_body_width_m=2.0,
            rear_joint_to_bumper_m=articulation_to_rear + rear_overhang,
            rear_body_width_m=2.0,
        )
        assert traj.rear_footprint_series is not None
        rear0 = traj.rear_footprint_series[0]  # (4, 2) corners of the first sample
        min_x = rear0[:, 0].min()
        assert min_x == pytest.approx(-(articulation_to_rear + rear_overhang))
        assert min_x < -articulation_to_rear  # extends behind the rear axle

    def test_single_bicycle_footprint_extends_ahead_of_front_axle(self):
        # Pose reference is the rear axle, so front_overhang_m = wheelbase + overhang. Regression
        # guard: a wheelbase fraction used to put the front bumper behind the front axle.
        wheelbase = 2.5
        front_overhang = 0.6
        rear_overhang = 0.5
        traj = single_bicycle_trajectory(
            wheelbase=wheelbase,
            track_width=1.5,
            wheel_radius=0.3,
            initial_steering_angle_rad=0.0,
            target_steering_angle_rad=0.0,
            steering_rate_rad_s=0.0,
            drive_velocity=0.0,  # stay at the origin so the geometry is exact
            duration=0.2,
            time_step=0.1,
            front_overhang_m=wheelbase + front_overhang,
            rear_overhang_m=rear_overhang,
            body_width_m=1.5,
        )
        assert traj.footprint_series is not None
        body0 = traj.footprint_series[0]  # (4, 2) corners of the first sample
        max_x = body0[:, 0].max()
        min_x = body0[:, 0].min()
        assert max_x == pytest.approx(wheelbase + front_overhang)
        assert max_x > wheelbase  # front bumper is AHEAD of the front axle
        assert min_x == pytest.approx(-rear_overhang)
        assert max_x - min_x == pytest.approx(wheelbase + front_overhang + rear_overhang)

    def test_single_differential_footprint_straddles_body_centre(self):
        # Pose reference is the body centre, so the overhangs are the bumper distances directly.
        front_overhang = 0.4
        rear_overhang = 0.3
        traj = single_differential_trajectory(
            wheel_radius=0.1,
            track_width=0.5,
            initial_linear_velocity=0.0,
            initial_angular_velocity=0.0,
            target_linear_velocity=0.0,
            target_angular_velocity=0.0,
            linear_acceleration=0.0,
            angular_acceleration=0.0,
            duration=0.2,
            time_step=0.1,
            front_overhang_m=front_overhang,
            rear_overhang_m=rear_overhang,
            body_width_m=0.5,
        )
        assert traj.footprint_series is not None
        body0 = traj.footprint_series[0]
        assert body0[:, 0].max() == pytest.approx(front_overhang)
        assert body0[:, 0].min() == pytest.approx(-rear_overhang)
