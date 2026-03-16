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
    BicycleModel,
    DifferentialDriveModel,
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
