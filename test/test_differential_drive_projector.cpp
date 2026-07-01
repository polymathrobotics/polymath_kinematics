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
constexpr double WHEEL_RADIUS = 0.1;
constexpr double TRACK_WIDTH = 0.5;
constexpr double V_MIN = -2.0;
constexpr double V_MAX = 2.0;
constexpr double OMEGA_MIN = -3.0;
constexpr double OMEGA_MAX = 3.0;

DifferentialDriveProjector makeProjector()
{
  return DifferentialDriveProjector(
    DifferentialDriveModel(WHEEL_RADIUS, TRACK_WIDTH), V_MIN, V_MAX, OMEGA_MIN, OMEGA_MAX);
}
}  // namespace

TEST_CASE("DifferentialDriveProjector construction stores model and limits")
{
  auto projector = makeProjector();
  CHECK(projector.get_min_linear_velocity_m_s() == Approx(V_MIN));
  CHECK(projector.get_max_linear_velocity_m_s() == Approx(V_MAX));
  CHECK(projector.get_min_angular_velocity_rad_s() == Approx(OMEGA_MIN));
  CHECK(projector.get_max_angular_velocity_rad_s() == Approx(OMEGA_MAX));
  CHECK(projector.get_model().get_wheel_radius_m() == Approx(WHEEL_RADIUS));
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
  CHECK(result.linear_velocity_m_s == Approx(V_MAX));
  CHECK(result.angular_velocity_rad_s == Approx(OMEGA_MAX));
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

TEST_CASE("DifferentialDriveProjector - no footprint dims yields an empty footprint (never throws)")
{
  auto projector = makeProjector();
  Pose2D pose{0.0, 0.0, 0.0};
  auto result = projector.step(0.1, pose, 0.0, 0.0, 0.0, 0.0, 100.0, 100.0);
  CHECK(result.footprint.empty());
}

TEST_CASE("DifferentialDriveProjector footprint - rectangle corners rotated by heading")
{
  constexpr double FRONT_OVERHANG = 1.5;
  constexpr double REAR_OVERHANG = 0.5;
  constexpr double BODY_WIDTH = 1.0;
  DifferentialDriveProjector projector(
    DifferentialDriveModel(WHEEL_RADIUS, TRACK_WIDTH),
    V_MIN,
    V_MAX,
    OMEGA_MIN,
    OMEGA_MAX,
    FRONT_OVERHANG,
    REAR_OVERHANG,
    BODY_WIDTH);

  // Heading = +90 deg (theta = pi/2), v=0 so pose stays at the origin.
  const double theta = M_PI / 2.0;
  Pose2D pose{0.0, 0.0, theta};
  auto result = projector.step(0.1, pose, 0.0, 0.0, 0.0, 0.0, 100.0, 100.0);
  REQUIRE(result.footprint.size() == 4);
  // Body-frame front-right corner (FRONT_OVERHANG, -BODY_WIDTH/2) rotated by +90deg:
  // world = (+BODY_WIDTH/2, FRONT_OVERHANG).
  CHECK(result.footprint[1].x == Approx(BODY_WIDTH / 2.0));
  CHECK(result.footprint[1].y == Approx(FRONT_OVERHANG));
  // Body-frame rear-right corner (-REAR_OVERHANG, -BODY_WIDTH/2) rotated by +90deg:
  // world = (+BODY_WIDTH/2, -REAR_OVERHANG).
  CHECK(result.footprint[0].x == Approx(BODY_WIDTH / 2.0));
  CHECK(result.footprint[0].y == Approx(-REAR_OVERHANG));
}

}  // namespace polymath::kinematics
