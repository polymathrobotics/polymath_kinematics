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

#include "catch2_compat.hpp"
#include "polymath_kinematics/differential_drive_model.hpp"

namespace polymath::kinematics
{

TEST_CASE("DifferentialDriveModel construction")
{
  DifferentialDriveModel model(0.1, 0.5);
  CHECK(model.get_wheel_radius_m() == 0.1);
  CHECK(model.get_track_width_m() == 0.5);
}

TEST_CASE("DifferentialDriveModel wheelVelocitiesToBodyVelocity - straight line")
{
  DifferentialDriveModel model(0.1, 0.5);  // 0.1m wheel radius, 0.5m track width

  // Both wheels spinning at same rate -> straight line motion
  auto result = model.wheelVelocitiesToBodyVelocity(10.0, 10.0);

  // Linear velocity = wheel_radius * wheel_vel = 0.1 * 10 = 1.0 m/s
  CHECK(result.linear_velocity_m_s == Approx(1.0));
  // Angular velocity = 0 when both wheels spin at same rate
  CHECK(result.angular_velocity_rad_s == Approx(0.0));
}

TEST_CASE("DifferentialDriveModel wheelVelocitiesToBodyVelocity - turning")
{
  DifferentialDriveModel model(0.1, 0.5);

  // Right wheel spinning faster -> turning left (positive angular velocity)
  auto result = model.wheelVelocitiesToBodyVelocity(5.0, 15.0);

  // Linear velocity = (0.1 * 5 + 0.1 * 15) / 2 = (0.5 + 1.5) / 2 = 1.0 m/s
  CHECK(result.linear_velocity_m_s == Approx(1.0));
  // Angular velocity = (1.5 - 0.5) / 0.5 = 2.0 rad/s
  CHECK(result.angular_velocity_rad_s == Approx(2.0));
}

TEST_CASE("DifferentialDriveModel wheelVelocitiesToBodyVelocity - spinning in place")
{
  DifferentialDriveModel model(0.1, 0.5);

  // Wheels spinning opposite directions -> spin in place
  auto result = model.wheelVelocitiesToBodyVelocity(-10.0, 10.0);

  // Linear velocity = (-1.0 + 1.0) / 2 = 0 m/s
  CHECK(result.linear_velocity_m_s == Approx(0.0));
  // Angular velocity = (1.0 - (-1.0)) / 0.5 = 4.0 rad/s
  CHECK(result.angular_velocity_rad_s == Approx(4.0));
}

TEST_CASE("DifferentialDriveModel bodyVelocityToWheelVelocities - straight line")
{
  DifferentialDriveModel model(0.1, 0.5);

  // Straight line at 1 m/s
  auto result = model.bodyVelocityToWheelVelocities(1.0, 0.0);

  // Both wheels should have same angular velocity
  // wheel_vel = linear_vel / wheel_radius = 1.0 / 0.1 = 10 rad/s
  CHECK(result.left_wheel_velocity_rad_s == Approx(10.0));
  CHECK(result.right_wheel_velocity_rad_s == Approx(10.0));
}

TEST_CASE("DifferentialDriveModel bodyVelocityToWheelVelocities - turning")
{
  DifferentialDriveModel model(0.1, 0.5);

  // 1 m/s forward with 2 rad/s angular velocity
  auto result = model.bodyVelocityToWheelVelocities(1.0, 2.0);

  // left_linear = 1.0 - 2.0 * 0.25 = 0.5 m/s -> 5 rad/s
  CHECK(result.left_wheel_velocity_rad_s == Approx(5.0));
  // right_linear = 1.0 + 2.0 * 0.25 = 1.5 m/s -> 15 rad/s
  CHECK(result.right_wheel_velocity_rad_s == Approx(15.0));
}

TEST_CASE("DifferentialDriveModel roundtrip")
{
  DifferentialDriveModel model(0.1, 0.5);

  double linear = 2.5;
  double angular = 1.0;

  auto wheel_state = model.bodyVelocityToWheelVelocities(linear, angular);
  auto velocities =
    model.wheelVelocitiesToBodyVelocity(wheel_state.left_wheel_velocity_rad_s, wheel_state.right_wheel_velocity_rad_s);

  CHECK(velocities.linear_velocity_m_s == Approx(linear));
  CHECK(velocities.angular_velocity_rad_s == Approx(angular));
}

}  // namespace polymath::kinematics
