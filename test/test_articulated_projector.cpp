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
#include "polymath_kinematics/articulated_projector.hpp"

namespace polymath::kinematics
{

namespace
{
// stueve-style parameters
constexpr double kFrontArm = 1.66;
constexpr double kRearArm = 1.44;
constexpr double kFrontTrack = 2.0;
constexpr double kRearTrack = 2.0;
constexpr double kFrontWheelRadius = 0.723;
constexpr double kRearWheelRadius = 0.723;
constexpr double kMinAngle = -0.785;
constexpr double kMaxAngle = 0.785;

ArticulatedModel make_model()
{
  return ArticulatedModel(kFrontArm, kRearArm, kFrontTrack, kRearTrack, kFrontWheelRadius, kRearWheelRadius);
}
}  // namespace

TEST_CASE("ArticulatedProjector construction stores model and limits")
{
  ArticulatedProjector projector(make_model(), kMinAngle, kMaxAngle);
  CHECK(projector.get_min_articulation_angle_rad() == Approx(kMinAngle));
  CHECK(projector.get_max_articulation_angle_rad() == Approx(kMaxAngle));
  CHECK(projector.get_model().get_articulation_to_front_axle_m() == Approx(kFrontArm));
}

TEST_CASE("ArticulatedProjector step - zero rate freezes articulation angle")
{
  ArticulatedProjector projector(make_model(), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.3, 0.6, 0.0, 1.0);
  CHECK(result.articulation_angle_rad == Approx(0.3));
}

TEST_CASE("ArticulatedProjector step - large rate snaps to target without overshoot")
{
  ArticulatedProjector projector(make_model(), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.0, 0.4, 100.0, 1.0);
  CHECK(result.articulation_angle_rad == Approx(0.4));
}

TEST_CASE("ArticulatedProjector step - rate-limited slew advances by rate*dt")
{
  ArticulatedProjector projector(make_model(), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  // rate*dt = 0.02, target far away
  auto result = projector.step(0.1, pose, 0.0, 0.5, 0.2, 1.0);
  CHECK(result.articulation_angle_rad == Approx(0.02));
}

TEST_CASE("ArticulatedProjector step - target above max saturates at max")
{
  ArticulatedProjector projector(make_model(), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.0, 2.0, 100.0, 1.0);
  CHECK(result.articulation_angle_rad == Approx(kMaxAngle));
}

TEST_CASE("ArticulatedProjector step - negative articulation rate is treated as magnitude")
{
  ArticulatedProjector projector(make_model(), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.0, 0.5, -0.2, 1.0);
  CHECK(result.articulation_angle_rad == Approx(0.02));
}

TEST_CASE("ArticulatedProjector project - straight line: zero articulation, theta unchanged")
{
  ArticulatedProjector projector(make_model(), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  auto trajectory = projector.project(1.0, 0.1, pose, 0.0, 0.0, 0.0, 1.5);
  CHECK_EQ(trajectory.size(), 11);
  CHECK_EQ(trajectory.back().pose.x, Approx(1.5));
  CHECK_EQ(trajectory.back().pose.y, Approx(0.0));
  CHECK_EQ(trajectory.back().pose.theta, Approx(0.0));
  CHECK_EQ(trajectory.back().articulation_angle_rad, Approx(0.0));
}

TEST_CASE("ArticulatedProjector project - articulation ramps then saturates at max")
{
  ArticulatedProjector projector(make_model(), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  // target = 0.785 (= max), rate = 0.2 rad/s, dt = 0.1 → step adds 0.02
  // Reach max in ceil(0.785 / 0.02) = 40 steps; horizon 5.0s → 50 steps total.
  auto trajectory = projector.project(5.0, 0.1, pose, 0.0, 0.785, 0.2, 1.0);
  CHECK_EQ(trajectory.size(), 51);
  CHECK_EQ(trajectory[40].articulation_angle_rad, Approx(0.785));
  CHECK_EQ(trajectory.back().articulation_angle_rad, Approx(0.785));
}

TEST_CASE("ArticulatedProjector project - initial state stored as element 0")
{
  ArticulatedProjector projector(make_model(), kMinAngle, kMaxAngle);
  Pose2D pose{1.0, 2.0, 0.5};

  auto trajectory = projector.project(0.5, 0.1, pose, 0.2, 0.6, 0.5, 1.0);
  CHECK(trajectory.front().time_s == Approx(0.0));
  CHECK(trajectory.front().pose.x == Approx(1.0));
  CHECK(trajectory.front().pose.y == Approx(2.0));
  CHECK(trajectory.front().pose.theta == Approx(0.5));
  CHECK(trajectory.front().articulation_angle_rad == Approx(0.2));
}

TEST_CASE("ArticulatedProjector step - heading rotates with non-zero articulation at steady state")
{
  ArticulatedProjector projector(make_model(), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  // With current == clamped target, the realized gamma-dot is zero (no joint motion this step),
  // so the rear-axle turning velocity is purely a function of v and the articulation angle.
  // Positive articulation + forward v → positive (CCW) rear-axle omega and theta increase.
  auto result = projector.step(0.1, pose, 0.3, 0.3, 100.0, 1.0);
  CHECK(result.articulation_angle_rad == Approx(0.3));
  CHECK(result.angular_velocity_rad_s > 0.0);
  CHECK(result.pose.theta > 0.0);
}

TEST_CASE("ArticulatedProjector step - realized gamma-dot affects rear-axle omega")
{
  ArticulatedProjector projector(make_model(), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  // Same starting angle, same target, same v: stepping with a small rate (slow ramp) versus
  // a large rate (snap) yields different rear-axle angular velocities because gamma-dot
  // contributes to the kinematics.
  auto slow_ramp = projector.step(0.1, pose, 0.0, 0.3, 0.2, 1.0);  // step adds 0.02 (small gamma-dot)
  auto snap = projector.step(0.1, pose, 0.0, 0.3, 100.0, 1.0);  // snaps to 0.3 (huge gamma-dot)
  CHECK(slow_ramp.angular_velocity_rad_s != Approx(snap.angular_velocity_rad_s));
}

}  // namespace polymath::kinematics
