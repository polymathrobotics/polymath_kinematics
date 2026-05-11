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

import math

import pytest

from polymath_kinematics import (
    ArticulatedModel,
    ArticulatedProjector,
    BicycleModel,
    BicycleProjector,
    DifferentialDriveModel,
    Pose2D,
)


class TestDifferentialDriveModel:
    def test_construction(self):
        model = DifferentialDriveModel(0.1, 0.5)
        assert model.wheel_radius == pytest.approx(0.1)
        assert model.track_width == pytest.approx(0.5)

    def test_turning(self):
        model = DifferentialDriveModel(0.1, 0.5)
        result = model.wheel_velocities_to_body_velocity(5.0, 15.0)

        assert result.linear_velocity_m_s == pytest.approx(1.0)
        assert result.angular_velocity_rad_s == pytest.approx(2.0)

    def test_roundtrip(self):
        model = DifferentialDriveModel(0.1, 0.5)
        linear = 2.5
        angular = 1.0

        wheel_state = model.body_velocity_to_wheel_velocities(linear, angular)
        velocities = model.wheel_velocities_to_body_velocity(
            wheel_state.left_wheel_velocity_rad_s,
            wheel_state.right_wheel_velocity_rad_s,
        )

        assert velocities.linear_velocity_m_s == pytest.approx(linear)
        assert velocities.angular_velocity_rad_s == pytest.approx(angular)


class TestBicycleModel:
    def test_construction(self):
        model = BicycleModel(2.5, 1.5, 0.3)
        assert model.wheelbase == pytest.approx(2.5)
        assert model.track_width == pytest.approx(1.5)
        assert model.wheel_radius == pytest.approx(0.3)

    def test_turning(self):
        model = BicycleModel(2.5, 1.5, 0.3)
        result = model.body_velocity_to_steering(5.0, 2.0)

        assert result.velocity_m_s == pytest.approx(5.0)
        assert result.steering_angle_rad == pytest.approx(math.pi / 4)
        assert result.turning_radius_m == pytest.approx(2.5)

    def test_steering_angle_from_radius_negative(self):
        model = BicycleModel(2.5, 1.5, 0.3)
        angle = model.steering_angle_from_radius(-2.5)
        assert angle == pytest.approx(-math.pi / 4)

    def test_roundtrip(self):
        model = BicycleModel(2.5, 1.5, 0.3)
        velocity = 3.0
        steering_angle = -0.3

        body_vel = model.steering_to_body_velocity(velocity, steering_angle)
        steering = model.body_velocity_to_steering(body_vel.linear_velocity_m_s, body_vel.angular_velocity_rad_s)

        assert steering.velocity_m_s == pytest.approx(velocity)
        assert steering.steering_angle_rad == pytest.approx(steering_angle)

    def test_reverse_turning(self):
        model = BicycleModel(2.5, 1.5, 0.3)
        # Reversing with CCW body rotation → front wheels steer RIGHT (negative delta)
        result = model.body_velocity_to_steering(-5.0, 2.0)
        assert result.velocity_m_s == pytest.approx(-5.0)
        assert result.steering_angle_rad == pytest.approx(-math.pi / 4)

    def test_reverse_roundtrip(self):
        model = BicycleModel(2.5, 1.5, 0.3)
        velocity = -3.0
        steering_angle = -0.3

        body_vel = model.steering_to_body_velocity(velocity, steering_angle)
        steering = model.body_velocity_to_steering(body_vel.linear_velocity_m_s, body_vel.angular_velocity_rad_s)

        assert steering.velocity_m_s == pytest.approx(velocity)
        assert steering.steering_angle_rad == pytest.approx(steering_angle)


class TestArticulatedModel:
    def test_construction(self):
        model = ArticulatedModel(1.5, 1.2, 1.8, 1.6, 0.4, 0.5)
        assert model.articulation_to_front_axle == pytest.approx(1.5)
        assert model.articulation_to_rear_axle == pytest.approx(1.2)
        assert model.front_track_width == pytest.approx(1.8)
        assert model.rear_track_width == pytest.approx(1.6)
        assert model.front_wheel_radius == pytest.approx(0.4)
        assert model.rear_wheel_radius == pytest.approx(0.5)

    def test_turning(self):
        model = ArticulatedModel(1.5, 1.2, 1.8, 1.6, 0.4, 0.5)
        result = model.body_velocity_to_vehicle_state(2.0, 0.5)

        assert result.linear_velocity_m_s == pytest.approx(2.0)
        assert result.articulation_angle_rad == pytest.approx(0.6435011088)
        assert result.front_axle_turning_radius_m == pytest.approx(4.0)

    def test_axle_velocities(self):
        model = ArticulatedModel(1.5, 1.2, 1.8, 1.6, 0.4, 0.5)
        result = model.articulation_to_axle_velocities(2.0, 0.3)

        assert result.linear_velocity_m_s == pytest.approx(2.0)
        assert result.front_axle_turning_velocity_rad_s == pytest.approx(0.2244737375)

    def test_reverse_turning(self):
        model = ArticulatedModel(1.5, 1.2, 1.8, 1.6, 0.4, 0.5)
        # Reversing (v < 0) + CCW yaw → articulation angle negative (mirrors forward case)
        result = model.body_velocity_to_vehicle_state(-2.0, 0.5)
        assert result.articulation_angle_rad == pytest.approx(-0.6435011088)

    def test_reverse_roundtrip(self):
        model = ArticulatedModel(1.5, 1.2, 1.8, 1.6, 0.4, 0.5)
        linear_velocity = -2.0
        angular_velocity = 0.5

        vehicle_state = model.body_velocity_to_vehicle_state(linear_velocity, angular_velocity)
        axle_vel = model.articulation_to_axle_velocities(linear_velocity, vehicle_state.articulation_angle_rad)

        assert axle_vel.front_axle_turning_velocity_rad_s == pytest.approx(angular_velocity, abs=1e-6)

    def test_axle_velocities_rate_defaults_to_zero(self):
        model = ArticulatedModel(1.5, 1.2, 1.8, 1.6, 0.4, 0.5)
        no_arg = model.articulation_to_axle_velocities(2.0, 0.3)
        explicit_zero = model.articulation_to_axle_velocities(2.0, 0.3, 0.0)

        assert no_arg.front_axle_turning_velocity_rad_s == pytest.approx(
            explicit_zero.front_axle_turning_velocity_rad_s
        )
        assert no_arg.rear_axle_turning_velocity_rad_s == pytest.approx(
            explicit_zero.rear_axle_turning_velocity_rad_s
        )

    def test_axle_velocities_nonzero_rate_changes_rear(self):
        model = ArticulatedModel(1.5, 1.2, 1.8, 1.6, 0.4, 0.5)
        gamma_dot = 0.25
        with_rate = model.articulation_to_axle_velocities(2.0, 0.3, gamma_dot)
        # rear-axle turning velocity = front-axle turning velocity - gamma_dot, exactly.
        assert (
            with_rate.front_axle_turning_velocity_rad_s - with_rate.rear_axle_turning_velocity_rad_s
            == pytest.approx(gamma_dot)
        )


class TestBicycleProjector:
    def _make(self):
        return BicycleProjector(
            model=BicycleModel(2.5, 1.5, 0.3),
            min_steering_angle_rad=-0.6,
            max_steering_angle_rad=0.6,
        )

    def test_construction(self):
        projector = self._make()
        assert projector.min_steering_angle_rad == pytest.approx(-0.6)
        assert projector.max_steering_angle_rad == pytest.approx(0.6)
        assert projector.model.wheelbase == pytest.approx(2.5)

    def test_zero_rate_freezes_angle(self):
        projector = self._make()
        result = projector.step(
            dt_s=0.1,
            current_pose=Pose2D(),
            current_steering_angle_rad=0.2,
            target_steering_angle_rad=0.5,
            steering_rate_rad_s=0.0,
            linear_velocity_m_s=1.0,
        )
        assert result.steering_angle_rad == pytest.approx(0.2)

    def test_clamping_saturates_at_max(self):
        projector = self._make()
        result = projector.step(0.1, Pose2D(), 0.0, 5.0, 100.0, 1.0)
        assert result.steering_angle_rad == pytest.approx(0.6)

    def test_rate_limited_ramp(self):
        projector = self._make()
        result = projector.step(0.1, Pose2D(), 0.0, 0.5, 0.5, 1.0)
        assert result.steering_angle_rad == pytest.approx(0.05)

    def test_project_straight_line(self):
        projector = self._make()
        traj = projector.project(1.0, 0.1, Pose2D(), 0.0, 0.0, 1.0, 2.0)
        assert len(traj) == 11
        assert traj[0].time_s == pytest.approx(0.0)
        assert traj[-1].time_s == pytest.approx(1.0)
        assert traj[-1].pose.x == pytest.approx(2.0)
        assert traj[-1].pose.y == pytest.approx(0.0)
        assert traj[-1].pose.theta == pytest.approx(0.0)

    def test_project_ramps_to_target(self):
        projector = self._make()
        # target=0.5, rate=0.5, dt=0.1 → step adds 0.05; reaches 0.5 at step 10.
        traj = projector.project(2.0, 0.1, Pose2D(), 0.0, 0.5, 0.5, 0.0)
        assert traj[10].steering_angle_rad == pytest.approx(0.5)
        assert traj[-1].steering_angle_rad == pytest.approx(0.5)


class TestArticulatedProjector:
    def _make(self):
        return ArticulatedProjector(
            model=ArticulatedModel(1.66, 1.44, 2.0, 2.0, 0.723, 0.723),
            min_articulation_angle_rad=-0.785,
            max_articulation_angle_rad=0.785,
        )

    def test_construction(self):
        projector = self._make()
        assert projector.min_articulation_angle_rad == pytest.approx(-0.785)
        assert projector.max_articulation_angle_rad == pytest.approx(0.785)
        assert projector.model.articulation_to_front_axle == pytest.approx(1.66)

    def test_zero_rate_freezes_angle(self):
        projector = self._make()
        result = projector.step(0.1, Pose2D(), 0.3, 0.6, 0.0, 1.0)
        assert result.articulation_angle_rad == pytest.approx(0.3)

    def test_clamping_saturates_at_max(self):
        projector = self._make()
        result = projector.step(0.1, Pose2D(), 0.0, 2.0, 100.0, 1.0)
        assert result.articulation_angle_rad == pytest.approx(0.785)

    def test_rate_limited_ramp(self):
        projector = self._make()
        result = projector.step(0.1, Pose2D(), 0.0, 0.5, 0.2, 1.0)
        assert result.articulation_angle_rad == pytest.approx(0.02)

    def test_project_stueve_max_articulation(self):
        projector = self._make()
        # target=0.785, rate=0.2 rad/s, dt=0.1 → 0.02 per step; reaches max at step 40 (~3.925s ceil to 4.0s).
        traj = projector.project(5.0, 0.1, Pose2D(), 0.0, 0.785, 0.2, 1.0)
        assert len(traj) == 51
        assert traj[40].articulation_angle_rad == pytest.approx(0.785)
        assert traj[-1].articulation_angle_rad == pytest.approx(0.785)

    def test_project_initial_state_anchored(self):
        projector = self._make()
        initial = Pose2D(x=1.0, y=2.0, theta=0.5)
        traj = projector.project(0.5, 0.1, initial, 0.2, 0.6, 0.5, 1.0)
        assert traj[0].time_s == pytest.approx(0.0)
        assert traj[0].pose.x == pytest.approx(1.0)
        assert traj[0].pose.y == pytest.approx(2.0)
        assert traj[0].pose.theta == pytest.approx(0.5)
        assert traj[0].articulation_angle_rad == pytest.approx(0.2)
