// Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <limits>

#include "catch2_compat.hpp"
#include "polymath_kinematics/articulated_model.hpp"

namespace polymath::kinematics
{

TEST_CASE("ArticulatedModel construction")
{
  ArticulatedModel model(1.5, 1.2, 1.8, 1.6, 0.4, 0.5);

  CHECK(model.get_articulation_to_front_axle_m() == Approx(1.5));
  CHECK(model.get_articulation_to_rear_axle_m() == Approx(1.2));
  CHECK(model.get_front_track_width_m() == Approx(1.8));
  CHECK(model.get_rear_track_width_m() == Approx(1.6));
  CHECK(model.get_front_wheel_radius_m() == Approx(0.4));
  CHECK(model.get_rear_wheel_radius_m() == Approx(0.5));
}

TEST_CASE("ArticulatedModel articulationToAxleVelocities - straight line")
{
  ArticulatedModel model(1.5, 1.2, 1.8, 1.6, 0.4, 0.5);

  auto result = model.articulationToAxleVelocities(2.0, 0.0);

  CHECK(result.linear_velocity_m_s == Approx(2.0));
  CHECK(result.front_axle_turning_velocity_rad_s == Approx(0.0).margin(1e-6));
  CHECK(result.rear_axle_turning_velocity_rad_s == Approx(0.0).margin(1e-6));
}

TEST_CASE("ArticulatedModel articulationToAxleVelocities - turning")
{
  ArticulatedModel model(1.5, 1.2, 1.8, 1.6, 0.4, 0.5);

  // front_omega = v * sin(gamma) / (Lf * cos(gamma) + Lr)
  //             = 2.0 * sin(0.3) / (1.5 * cos(0.3) + 1.2)
  auto result = model.articulationToAxleVelocities(2.0, 0.3);

  CHECK(result.linear_velocity_m_s == Approx(2.0));
  CHECK(result.front_axle_turning_velocity_rad_s == Approx(0.2244737375));
  CHECK(result.rear_axle_turning_velocity_rad_s == Approx(0.2244737375));
}

TEST_CASE("ArticulatedModel bodyVelocityToVehicleState - left turn")
{
  ArticulatedModel model(1.5, 1.2, 1.8, 1.6, 0.4, 0.5);

  auto result = model.bodyVelocityToVehicleState(2.0, 0.5);

  CHECK(result.linear_velocity_m_s == Approx(2.0));
  CHECK(result.articulation_angle_rad == Approx(0.6435011088));
  CHECK(result.front_axle_turning_radius_m == Approx(4.0));
  CHECK(result.rear_axle_turning_radius_m == Approx(4.1));

  // Inner wheels (left for left turn) should be slower than outer
  CHECK(result.front_left_wheel_speed_rad_s < result.front_right_wheel_speed_rad_s);
  CHECK(result.rear_left_wheel_speed_rad_s < result.rear_right_wheel_speed_rad_s);

  CHECK(result.front_right_wheel_speed_rad_s == Approx(6.125));
  CHECK(result.front_left_wheel_speed_rad_s == Approx(3.875));
  CHECK(result.rear_right_wheel_speed_rad_s == Approx(4.9));
  CHECK(result.rear_left_wheel_speed_rad_s == Approx(3.3));
}

TEST_CASE("ArticulatedModel bodyVelocityToVehicleState - right turn")
{
  ArticulatedModel model(1.5, 1.2, 1.8, 1.6, 0.4, 0.5);

  auto result = model.bodyVelocityToVehicleState(2.0, -0.5);

  CHECK(result.linear_velocity_m_s == Approx(2.0));
  CHECK(result.articulation_angle_rad == Approx(-0.6435011088));

  // All wheel speeds should be positive (vehicle moving forward)
  CHECK(result.front_right_wheel_speed_rad_s > 0.0);
  CHECK(result.front_left_wheel_speed_rad_s > 0.0);
  CHECK(result.rear_right_wheel_speed_rad_s > 0.0);
  CHECK(result.rear_left_wheel_speed_rad_s > 0.0);

  // Inner wheels (right for right turn) should be slower
  CHECK(result.front_right_wheel_speed_rad_s < result.front_left_wheel_speed_rad_s);
  CHECK(result.rear_right_wheel_speed_rad_s < result.rear_left_wheel_speed_rad_s);
}

TEST_CASE("ArticulatedModel roundtrip - bodyVelocityToVehicleState to articulationToAxleVelocities")
{
  ArticulatedModel model(1.5, 1.2, 1.8, 1.6, 0.4, 0.5);

  double linear_velocity = 2.0;
  double angular_velocity = 0.5;

  // Get the articulation angle from body velocity
  auto vehicle_state = model.bodyVelocityToVehicleState(linear_velocity, angular_velocity);

  // Feed that articulation angle back into articulationToAxleVelocities
  auto axle_vel = model.articulationToAxleVelocities(linear_velocity, vehicle_state.articulation_angle_rad);

  // The front axle turning velocity magnitude should match the body angular velocity.
  // Sign is negated: positive body omega produces negative front axle turning velocity
  // (the front axle turns opposite to the body yaw direction).
  CHECK(axle_vel.front_axle_turning_velocity_rad_s == Approx(angular_velocity).margin(1e-6));
}

TEST_CASE("ArticulatedModel bodyVelocityToVehicleState - stationary (0, 0) no NaN")
{
  // sqrt(omega^2 * Lf^2 + v^2) = 0 produces 0/0 in acos — guard must return zeros
  ArticulatedModel model(1.5, 1.2, 1.8, 1.6, 0.4, 0.5);

  auto result = model.bodyVelocityToVehicleState(0.0, 0.0);

  CHECK(result.articulation_angle_rad == Approx(0.0));
  CHECK(result.linear_velocity_m_s == Approx(0.0));
  CHECK(result.front_right_wheel_speed_rad_s == Approx(0.0));
  CHECK(result.front_left_wheel_speed_rad_s == Approx(0.0));
  CHECK(result.rear_right_wheel_speed_rad_s == Approx(0.0));
  CHECK(result.rear_left_wheel_speed_rad_s == Approx(0.0));
  CHECK(std::isinf(result.front_axle_turning_radius_m));
  CHECK(std::isinf(result.rear_axle_turning_radius_m));
}

TEST_CASE("ArticulatedModel bodyVelocityToVehicleState - straight line (v, 0) no NaN")
{
  // articulation_angle = 0, so sin(0) = 0 -> radii = inf -> inf * 0 = NaN wheel speeds without guard
  ArticulatedModel model(1.5, 1.2, 1.8, 1.6, 0.4, 0.5);

  auto result = model.bodyVelocityToVehicleState(2.0, 0.0);

  CHECK(result.articulation_angle_rad == Approx(0.0));
  CHECK(result.linear_velocity_m_s == Approx(2.0));
  CHECK(result.front_right_wheel_speed_rad_s == Approx(2.0 / 0.4));
  CHECK(result.front_left_wheel_speed_rad_s == Approx(2.0 / 0.4));
  CHECK(result.rear_right_wheel_speed_rad_s == Approx(2.0 / 0.5));
  CHECK(result.rear_left_wheel_speed_rad_s == Approx(2.0 / 0.5));
  CHECK(std::isinf(result.front_axle_turning_radius_m));
  CHECK(std::isinf(result.rear_axle_turning_radius_m));
}

TEST_CASE("ArticulatedModel bodyVelocityToVehicleState - reverse left turn")
{
  ArticulatedModel model(1.5, 1.2, 1.8, 1.6, 0.4, 0.5);

  // Reversing (v < 0) + CCW yaw (omega > 0): articulation angle must be negative (joint bends right)
  // |gamma| matches the forward case by symmetry: gamma = -0.6435011088
  auto result = model.bodyVelocityToVehicleState(-2.0, 0.5);

  CHECK(result.linear_velocity_m_s == Approx(-2.0));
  CHECK(result.articulation_angle_rad == Approx(-0.6435011088));
  CHECK(result.front_axle_turning_radius_m == Approx(-4.0));
  CHECK(result.rear_axle_turning_radius_m == Approx(-4.1));

  // All wheel speeds are negative (reversing)
  CHECK(result.front_right_wheel_speed_rad_s < 0.0);
  CHECK(result.front_left_wheel_speed_rad_s < 0.0);
  CHECK(result.rear_right_wheel_speed_rad_s < 0.0);
  CHECK(result.rear_left_wheel_speed_rad_s < 0.0);

  // Right wheels are closer to ICR (smaller magnitude) than left wheels
  CHECK(std::abs(result.front_right_wheel_speed_rad_s) < std::abs(result.front_left_wheel_speed_rad_s));
  CHECK(std::abs(result.rear_right_wheel_speed_rad_s) < std::abs(result.rear_left_wheel_speed_rad_s));
}

TEST_CASE("ArticulatedModel bodyVelocityToVehicleState - reverse right turn")
{
  ArticulatedModel model(1.5, 1.2, 1.8, 1.6, 0.4, 0.5);

  // Reversing (v < 0) + CW yaw (omega < 0): articulation angle must be positive (joint bends left)
  auto result = model.bodyVelocityToVehicleState(-2.0, -0.5);

  CHECK(result.linear_velocity_m_s == Approx(-2.0));
  CHECK(result.articulation_angle_rad == Approx(0.6435011088));

  // All wheel speeds are negative (reversing)
  CHECK(result.front_right_wheel_speed_rad_s < 0.0);
  CHECK(result.front_left_wheel_speed_rad_s < 0.0);
  CHECK(result.rear_right_wheel_speed_rad_s < 0.0);
  CHECK(result.rear_left_wheel_speed_rad_s < 0.0);
}

TEST_CASE("ArticulatedModel roundtrip reverse - bodyVelocityToVehicleState to articulationToAxleVelocities")
{
  ArticulatedModel model(1.5, 1.2, 1.8, 1.6, 0.4, 0.5);

  double linear_velocity = -2.0;
  double angular_velocity = 0.5;

  auto vehicle_state = model.bodyVelocityToVehicleState(linear_velocity, angular_velocity);
  auto axle_vel = model.articulationToAxleVelocities(linear_velocity, vehicle_state.articulation_angle_rad);

  CHECK(axle_vel.front_axle_turning_velocity_rad_s == Approx(angular_velocity).margin(1e-6));
}

}  // namespace polymath::kinematics
