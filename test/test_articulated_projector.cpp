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
constexpr double FRONT_ARM = 1.66;
constexpr double REAR_ARM = 1.44;
constexpr double FRONT_TRACK = 2.0;
constexpr double REAR_TRACK = 2.0;
constexpr double FRONT_WHEEL_RADIUS = 0.723;
constexpr double REAR_WHEEL_RADIUS = 0.723;
constexpr double MIN_ANGLE = -0.785;
constexpr double MAX_ANGLE = 0.785;

ArticulatedModel make_model()
{
  return ArticulatedModel(FRONT_ARM, REAR_ARM, FRONT_TRACK, REAR_TRACK, FRONT_WHEEL_RADIUS, REAR_WHEEL_RADIUS);
}
}  // namespace

TEST_CASE("ArticulatedProjector construction stores model and limits")
{
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
  CHECK(projector.get_min_articulation_angle_rad() == Approx(MIN_ANGLE));
  CHECK(projector.get_max_articulation_angle_rad() == Approx(MAX_ANGLE));
  CHECK(projector.get_model().get_articulation_to_front_axle_m() == Approx(FRONT_ARM));
}

TEST_CASE("ArticulatedProjector step - zero rate freezes articulation angle")
{
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.3, 0.6, 0.0, 1.0);
  CHECK(result.articulation_angle_rad == Approx(0.3));
}

TEST_CASE("ArticulatedProjector step - large rate snaps to target without overshoot")
{
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.0, 0.4, 100.0, 1.0);
  CHECK(result.articulation_angle_rad == Approx(0.4));
}

TEST_CASE("ArticulatedProjector step - rate-limited slew advances by rate*dt")
{
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  // rate*dt = 0.02, target far away
  auto result = projector.step(0.1, pose, 0.0, 0.5, 0.2, 1.0);
  CHECK(result.articulation_angle_rad == Approx(0.02));
}

TEST_CASE("ArticulatedProjector step - target above max saturates at max")
{
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.0, 2.0, 100.0, 1.0);
  CHECK(result.articulation_angle_rad == Approx(MAX_ANGLE));
}

TEST_CASE("ArticulatedProjector step - negative articulation rate is treated as magnitude")
{
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.0, 0.5, -0.2, 1.0);
  CHECK(result.articulation_angle_rad == Approx(0.02));
}

TEST_CASE("ArticulatedProjector project - straight line: zero articulation, theta unchanged")
{
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  auto trajectory = projector.project(1.0, 0.1, pose, 0.0, 0.0, 0.0, 1.5);
  CHECK(trajectory.size() == 11);
  CHECK(trajectory.back().pose.x == Approx(1.5));
  CHECK(trajectory.back().pose.y == Approx(0.0));
  CHECK(trajectory.back().pose.theta == Approx(0.0));
  CHECK(trajectory.back().articulation_angle_rad == Approx(0.0));
}

TEST_CASE("ArticulatedProjector project - articulation ramps then saturates at max")
{
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  // target = 0.785 (= max), rate = 0.2 rad/s, dt = 0.1 → step adds 0.02
  // Reach max in ceil(0.785 / 0.02) = 40 steps; horizon 5.0s → 50 steps total.
  auto trajectory = projector.project(5.0, 0.1, pose, 0.0, 0.785, 0.2, 1.0);
  CHECK(trajectory.size() == 51);
  CHECK(trajectory[40].articulation_angle_rad == Approx(0.785));
  CHECK(trajectory.back().articulation_angle_rad == Approx(0.785));
}

TEST_CASE("ArticulatedProjector project - initial state stored as element 0")
{
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
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
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
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
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  // Same starting angle, same target, same v: stepping with a small rate (slow ramp) versus
  // a large rate (snap) yields different rear-axle angular velocities because gamma-dot
  // contributes to the kinematics.
  auto slow_ramp = projector.step(0.1, pose, 0.0, 0.3, 0.2, 1.0);  // step adds 0.02 (small gamma-dot)
  auto snap = projector.step(0.1, pose, 0.0, 0.3, 100.0, 1.0);  // snaps to 0.3 (huge gamma-dot)
  CHECK(slow_ramp.angular_velocity_rad_s != Approx(snap.angular_velocity_rad_s));
}

// Footprint dimensions used by the footprint tests below.
namespace
{
constexpr double FRONT_JOINT_TO_BUMPER = 2.2;
constexpr double FRONT_BODY_WIDTH = 2.0;
constexpr double REAR_JOINT_TO_BUMPER = 2.0;
constexpr double REAR_BODY_WIDTH = 2.0;

ArticulatedProjector make_footprint_projector()
{
  return ArticulatedProjector(
    make_model(), MIN_ANGLE, MAX_ANGLE, FRONT_JOINT_TO_BUMPER, FRONT_BODY_WIDTH, REAR_JOINT_TO_BUMPER, REAR_BODY_WIDTH);
}
}  // namespace

TEST_CASE("ArticulatedProjector - no footprint dims yields empty footprints (never throws)")
{
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};
  auto result = projector.step(0.1, pose, 0.0, 0.0, 0.0, 1.0);
  CHECK(result.front_footprint.empty());
  CHECK(result.rear_footprint.empty());

  // project() must also run and produce states with empty footprints.
  auto states = projector.project(0.2, 0.1, pose, 0.0, 0.0, 0.0, 1.0);
  REQUIRE(states.size() >= 1);
  CHECK(states.front().front_footprint.empty());
  CHECK(states.front().rear_footprint.empty());
}

TEST_CASE("ArticulatedProjector footprint - zero articulation, both bodies aligned along +x")
{
  auto projector = make_footprint_projector();
  // Pose (base_link) is the articulation joint; place it at the origin, rear-body heading +x.
  Pose2D pose{0.0, 0.0, 0.0};
  auto result = projector.step(0.1, pose, 0.0, 0.0, 0.0, 0.0);  // v=0 so pose stays; gamma=0

  REQUIRE(result.front_footprint.size() == 4);
  REQUIRE(result.rear_footprint.size() == 4);

  // Joint is the pose (origin).
  const double joint_x = 0.0;
  // Front body spans [joint, joint + FRONT_JOINT_TO_BUMPER]; corners 0 and 3 are at the joint.
  CHECK(result.front_footprint[0].x == Approx(joint_x));
  CHECK(result.front_footprint[0].y == Approx(-FRONT_BODY_WIDTH / 2.0));
  CHECK(result.front_footprint[1].x == Approx(joint_x + FRONT_JOINT_TO_BUMPER));
  CHECK(result.front_footprint[1].y == Approx(-FRONT_BODY_WIDTH / 2.0));
  // Rear body spans [joint - REAR_JOINT_TO_BUMPER, joint].
  CHECK(result.rear_footprint[0].x == Approx(joint_x - REAR_JOINT_TO_BUMPER));
  CHECK(result.rear_footprint[1].x == Approx(joint_x));
  CHECK(result.rear_footprint[1].y == Approx(-REAR_BODY_WIDTH / 2.0));
}

TEST_CASE("ArticulatedProjector footprint - nonzero articulation folds the front body about the joint")
{
  auto projector = make_footprint_projector();
  Pose2D pose{0.0, 0.0, 0.0};
  const double gamma = 0.3;
  auto result = projector.step(0.1, pose, gamma, gamma, 0.0, 0.0);  // hold gamma, v=0

  REQUIRE(result.front_footprint.size() == 4);
  // Joint is the pose (origin).
  const double joint_x = 0.0;
  const double joint_y = 0.0;
  // The front-bumper midpoint (between corners 1 and 2) lies on the front-body axis at distance
  // FRONT_JOINT_TO_BUMPER from the joint, along the front heading (rear theta + gamma). Using the
  // midpoint avoids the body-width offset that each individual corner carries.
  const double front_theta = 0.0 + gamma;
  const double bumper_mid_x = (result.front_footprint[1].x + result.front_footprint[2].x) / 2.0;
  const double bumper_mid_y = (result.front_footprint[1].y + result.front_footprint[2].y) / 2.0;
  CHECK(bumper_mid_x == Approx(joint_x + FRONT_JOINT_TO_BUMPER * std::cos(front_theta)));
  CHECK(bumper_mid_y == Approx(joint_y + FRONT_JOINT_TO_BUMPER * std::sin(front_theta)));
  // Rear body stays aligned with the rear heading (unchanged from the zero case).
  CHECK(result.rear_footprint[0].x == Approx(joint_x - REAR_JOINT_TO_BUMPER));
  CHECK(result.rear_footprint[0].y == Approx(0.0 - REAR_BODY_WIDTH / 2.0));
}

}  // namespace polymath::kinematics
