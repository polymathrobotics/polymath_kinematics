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
#include "polymath_kinematics/differential_drive_projector.hpp"

namespace polymath::kinematics
{

namespace
{
constexpr double kWheelRadius = 0.1;
constexpr double kTrackWidth = 0.5;
constexpr double kVMin = -2.0;
constexpr double kVMax = 2.0;
constexpr double kOmegaMin = -3.0;
constexpr double kOmegaMax = 3.0;

DifferentialDriveProjector makeProjector()
{
  return DifferentialDriveProjector(
    DifferentialDriveModel(kWheelRadius, kTrackWidth), kVMin, kVMax, kOmegaMin, kOmegaMax);
}
}  // namespace

TEST_CASE("DifferentialDriveProjector construction stores model and limits")
{
  auto projector = makeProjector();
  CHECK(projector.get_min_linear_velocity_m_s() == Approx(kVMin));
  CHECK(projector.get_max_linear_velocity_m_s() == Approx(kVMax));
  CHECK(projector.get_min_angular_velocity_rad_s() == Approx(kOmegaMin));
  CHECK(projector.get_max_angular_velocity_rad_s() == Approx(kOmegaMax));
  CHECK(projector.get_model().get_wheel_radius_m() == Approx(kWheelRadius));
}

TEST_CASE("DifferentialDriveProjector step - zero acceleration freezes both velocities")
{
  auto projector = makeProjector();
  Pose2D pose{0.0, 0.0, 0.0};
  auto result = projector.step(0.1, pose, 0.5, 0.2, 1.5, 1.0, 0.0, 0.0);
  CHECK(result.linear_velocity_m_s == Approx(0.5));
  CHECK(result.angular_velocity_rad_s == Approx(0.2));
}

TEST_CASE("DifferentialDriveProjector step - large acceleration snaps to target without overshoot")
{
  auto projector = makeProjector();
  Pose2D pose{0.0, 0.0, 0.0};
  // |delta_v| = 1.0, |delta_omega| = 0.4 — much smaller than 100 * 0.1 = 10.0.
  auto result = projector.step(0.1, pose, 0.0, 0.0, 1.0, 0.4, 100.0, 100.0);
  CHECK(result.linear_velocity_m_s == Approx(1.0));
  CHECK(result.angular_velocity_rad_s == Approx(0.4));
}

TEST_CASE("DifferentialDriveProjector step - rate-limited ramps advance by accel*dt")
{
  auto projector = makeProjector();
  Pose2D pose{0.0, 0.0, 0.0};
  // accel*dt = 0.5 * 0.1 = 0.05 for linear; 0.3 * 0.1 = 0.03 for angular.
  auto result = projector.step(0.1, pose, 0.0, 0.0, 1.0, 0.5, 0.5, 0.3);
  CHECK(result.linear_velocity_m_s == Approx(0.05));
  CHECK(result.angular_velocity_rad_s == Approx(0.03));
}

TEST_CASE("DifferentialDriveProjector step - target above linear max saturates at max")
{
  auto projector = makeProjector();
  Pose2D pose{0.0, 0.0, 0.0};
  auto result = projector.step(0.1, pose, 0.0, 0.0, 100.0, 100.0, 1000.0, 1000.0);
  CHECK(result.linear_velocity_m_s == Approx(kVMax));
  CHECK(result.angular_velocity_rad_s == Approx(kOmegaMax));
}

TEST_CASE("DifferentialDriveProjector step - negative acceleration is treated as magnitude")
{
  auto projector = makeProjector();
  Pose2D pose{0.0, 0.0, 0.0};
  auto result = projector.step(0.1, pose, 0.0, 0.0, 1.0, 0.5, -0.5, -0.3);
  CHECK(result.linear_velocity_m_s == Approx(0.05));
  CHECK(result.angular_velocity_rad_s == Approx(0.03));
}

TEST_CASE("DifferentialDriveProjector project - straight line traces +x at constant v")
{
  auto projector = makeProjector();
  Pose2D pose{0.0, 0.0, 0.0};
  // Constant linear=1.0, zero angular, large accels so the ramp is irrelevant.
  auto trajectory = projector.project(1.0, 0.1, pose, 1.0, 0.0, 1.0, 0.0, 100.0, 100.0);
  CHECK(trajectory.size() == 11);
  CHECK(trajectory.front().time_s == Approx(0.0));
  CHECK(trajectory.back().time_s == Approx(1.0));
  CHECK(trajectory.back().pose.x == Approx(1.0));
  CHECK(trajectory.back().pose.y == Approx(0.0));
  CHECK(trajectory.back().pose.theta == Approx(0.0));
}

TEST_CASE("DifferentialDriveProjector project - initial state stored as element 0")
{
  auto projector = makeProjector();
  Pose2D pose{0.5, -1.0, 0.25};
  auto trajectory = projector.project(0.5, 0.1, pose, 0.2, 0.1, 0.5, 0.3, 0.5, 0.5);
  CHECK(trajectory.front().time_s == Approx(0.0));
  CHECK(trajectory.front().pose.x == Approx(0.5));
  CHECK(trajectory.front().pose.y == Approx(-1.0));
  CHECK(trajectory.front().pose.theta == Approx(0.25));
  CHECK(trajectory.front().linear_velocity_m_s == Approx(0.2));
  CHECK(trajectory.front().angular_velocity_rad_s == Approx(0.1));
}

TEST_CASE("DifferentialDriveProjector step - wheel speeds derive from body command")
{
  auto projector = makeProjector();
  Pose2D pose{0.0, 0.0, 0.0};
  // After snap, body command is (1.0 m/s, 0.0 rad/s); wheel speeds should both be 1.0 / 0.1 = 10 rad/s.
  auto result = projector.step(0.1, pose, 0.0, 0.0, 1.0, 0.0, 100.0, 100.0);
  CHECK(result.wheel_velocities.left_wheel_velocity_rad_s == Approx(10.0));
  CHECK(result.wheel_velocities.right_wheel_velocity_rad_s == Approx(10.0));
}

}  // namespace polymath::kinematics
