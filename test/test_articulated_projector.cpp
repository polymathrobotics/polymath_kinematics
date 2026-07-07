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

// Body footprints used by the footprint tests below. Each polygon is measured from its OWN axle.
namespace
{
// Front body: 2.2 m ahead of the front axle to the bucket, 0.4 m behind it.
constexpr double FRONT_AHEAD = 2.2;
constexpr double FRONT_BEHIND = 0.4;
constexpr double FRONT_BODY_WIDTH = 2.0;
// Rear body: 0.3 m ahead of the rear axle, 2.0 m behind it to the counterweight.
constexpr double REAR_AHEAD = 0.3;
constexpr double REAR_BEHIND = 2.0;
constexpr double REAR_BODY_WIDTH = 2.0;

ArticulatedProjector make_footprint_projector(AxleReference reference = AxleReference::REAR)
{
  return ArticulatedProjector(
    make_model(),
    MIN_ANGLE,
    MAX_ANGLE,
    reference,
    rectangleFootprint(FRONT_AHEAD, FRONT_BEHIND, FRONT_BODY_WIDTH),
    rectangleFootprint(REAR_AHEAD, REAR_BEHIND, REAR_BODY_WIDTH));
}
}  // namespace

TEST_CASE("ArticulatedProjector - no footprint set yields empty footprints (never throws)")
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

TEST_CASE("ArticulatedProjector - joint pose is reported alongside the reference-axle pose")
{
  auto projector = make_footprint_projector();
  // REAR reference: pose is the rear axle at the origin, so the joint sits REAR_ARM ahead.
  auto result = projector.step(0.1, Pose2D{0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0);
  CHECK(result.pose.x == Approx(0.0));
  CHECK(result.joint_pose.x == Approx(REAR_ARM));
  CHECK(result.joint_pose.y == Approx(0.0));
  CHECK(result.joint_pose.theta == Approx(0.0));
}

TEST_CASE("ArticulatedProjector footprint - zero articulation, each body about its own axle")
{
  auto projector = make_footprint_projector();
  // REAR reference: rear axle at the origin, rear-body heading +x. gamma=0 so both bodies align.
  auto result = projector.step(0.1, Pose2D{0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0);  // v=0 so pose stays

  REQUIRE(result.front_footprint.size() == 4);
  REQUIRE(result.rear_footprint.size() == 4);

  // Rear body is measured from the rear axle (the origin here).
  CHECK(result.rear_footprint[0].x == Approx(-REAR_BEHIND));
  CHECK(result.rear_footprint[0].y == Approx(-REAR_BODY_WIDTH / 2.0));
  CHECK(result.rear_footprint[1].x == Approx(REAR_AHEAD));

  // Front axle is a joint arm plus a front arm ahead of the rear axle when gamma = 0.
  const double front_axle_x = REAR_ARM + FRONT_ARM;
  CHECK(result.front_footprint[0].x == Approx(front_axle_x - FRONT_BEHIND));
  CHECK(result.front_footprint[0].y == Approx(-FRONT_BODY_WIDTH / 2.0));
  CHECK(result.front_footprint[1].x == Approx(front_axle_x + FRONT_AHEAD));
}

TEST_CASE("ArticulatedProjector footprint - nonzero articulation swings the front body about the joint")
{
  auto projector = make_footprint_projector();
  const double gamma = 0.3;
  auto result = projector.step(0.1, Pose2D{0.0, 0.0, 0.0}, gamma, gamma, 0.0, 0.0);  // hold gamma, v=0

  REQUIRE(result.front_footprint.size() == 4);
  // Joint sits REAR_ARM ahead of the rear axle; the front axle is FRONT_ARM further along the
  // front heading (theta_rear + gamma).
  const double front_theta = gamma;
  const double front_axle_x = REAR_ARM + FRONT_ARM * std::cos(front_theta);
  const double front_axle_y = FRONT_ARM * std::sin(front_theta);
  // Compare bumper midpoints, which lie on the body axis and carry no width offset.
  const double bumper_mid_x = (result.front_footprint[1].x + result.front_footprint[2].x) / 2.0;
  const double bumper_mid_y = (result.front_footprint[1].y + result.front_footprint[2].y) / 2.0;
  CHECK(bumper_mid_x == Approx(front_axle_x + FRONT_AHEAD * std::cos(front_theta)));
  CHECK(bumper_mid_y == Approx(front_axle_y + FRONT_AHEAD * std::sin(front_theta)));
  // Rear body is unaffected by gamma: still measured straight back from the rear axle.
  CHECK(result.rear_footprint[0].x == Approx(-REAR_BEHIND));
  CHECK(result.rear_footprint[0].y == Approx(-REAR_BODY_WIDTH / 2.0));
}

TEST_CASE("ArticulatedProjector - FRONT reference reports the front axle and its body heading")
{
  const double gamma = 0.3;
  auto projector = make_footprint_projector(AxleReference::FRONT);
  // Seed with the front-axle pose that corresponds to a rear axle at the origin heading +x.
  const double front_theta = gamma;
  const Pose2D front_start{
    REAR_ARM + FRONT_ARM * std::cos(front_theta), FRONT_ARM * std::sin(front_theta), front_theta};
  auto result = projector.step(0.1, front_start, gamma, gamma, 0.0, 0.0);  // hold gamma, v=0

  // Pose comes back at the front axle with the FRONT-body heading, and the joint is unchanged.
  CHECK(result.pose.x == Approx(front_start.x));
  CHECK(result.pose.y == Approx(front_start.y));
  CHECK(result.pose.theta == Approx(front_theta));
  CHECK(result.joint_pose.x == Approx(REAR_ARM));
  CHECK(result.joint_pose.y == Approx(0.0));
  CHECK(result.joint_pose.theta == Approx(0.0));
}

TEST_CASE("ArticulatedProjector - FRONT and REAR references describe the same physical motion")
{
  const double gamma = 0.25;
  auto rear_projector = make_footprint_projector(AxleReference::REAR);
  auto front_projector = make_footprint_projector(AxleReference::FRONT);

  const double front_theta = gamma;
  const Pose2D rear_start{0.0, 0.0, 0.0};
  const Pose2D front_start{
    REAR_ARM + FRONT_ARM * std::cos(front_theta), FRONT_ARM * std::sin(front_theta), front_theta};

  auto rear_traj = rear_projector.project(1.0, 0.05, rear_start, gamma, gamma, 0.0, 1.0);
  auto front_traj = front_projector.project(1.0, 0.05, front_start, gamma, gamma, 0.0, 1.0);
  REQUIRE(rear_traj.size() == front_traj.size());

  // Same vehicle: the joint pose and both world-frame footprints must agree sample for sample,
  // regardless of which axle the caller chose to reference.
  for (std::size_t i = 0; i < rear_traj.size(); ++i) {
    CHECK(front_traj[i].joint_pose.x == Approx(rear_traj[i].joint_pose.x));
    CHECK(front_traj[i].joint_pose.y == Approx(rear_traj[i].joint_pose.y));
    CHECK(front_traj[i].joint_pose.theta == Approx(rear_traj[i].joint_pose.theta));
    REQUIRE(front_traj[i].front_footprint.size() == rear_traj[i].front_footprint.size());
    for (std::size_t k = 0; k < rear_traj[i].front_footprint.size(); ++k) {
      CHECK(front_traj[i].front_footprint[k].x == Approx(rear_traj[i].front_footprint[k].x));
      CHECK(front_traj[i].front_footprint[k].y == Approx(rear_traj[i].front_footprint[k].y));
      CHECK(front_traj[i].rear_footprint[k].x == Approx(rear_traj[i].rear_footprint[k].x));
      CHECK(front_traj[i].rear_footprint[k].y == Approx(rear_traj[i].rear_footprint[k].y));
    }
  }
}

TEST_CASE("ArticulatedProjector footprint - arbitrary polygons are carried through vertex for vertex")
{
  // Five-vertex front body, three-vertex rear body: nothing may assume 4 corners.
  const Footprint front_body{
    Point2D{-0.4, -1.0}, Point2D{1.8, -1.0}, Point2D{2.4, 0.0}, Point2D{1.8, 1.0}, Point2D{-0.4, 1.0}};
  const Footprint rear_body{Point2D{0.3, -0.9}, Point2D{-1.9, 0.0}, Point2D{0.3, 0.9}};
  ArticulatedProjector projector(make_model(), MIN_ANGLE, MAX_ANGLE, AxleReference::REAR, front_body, rear_body);

  auto result = projector.step(0.1, Pose2D{0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0);
  REQUIRE(result.front_footprint.size() == front_body.size());
  REQUIRE(result.rear_footprint.size() == rear_body.size());
  // gamma = 0 and rear axle at the origin: rear body passes through unchanged, front body shifts
  // by the full wheelbase with no rotation.
  const double front_axle_x = REAR_ARM + FRONT_ARM;
  for (std::size_t i = 0; i < rear_body.size(); ++i) {
    CHECK(result.rear_footprint[i].x == Approx(rear_body[i].x));
    CHECK(result.rear_footprint[i].y == Approx(rear_body[i].y));
  }
  for (std::size_t i = 0; i < front_body.size(); ++i) {
    CHECK(result.front_footprint[i].x == Approx(front_axle_x + front_body[i].x));
    CHECK(result.front_footprint[i].y == Approx(front_body[i].y));
  }
}

}  // namespace polymath::kinematics
