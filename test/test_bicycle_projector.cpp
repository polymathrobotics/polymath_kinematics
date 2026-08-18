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
#include "polymath_kinematics/bicycle_projector.hpp"

namespace polymath::kinematics
{

namespace
{
constexpr double WHEELBASE = 2.5;
constexpr double TRACK = 1.5;
constexpr double WHEEL_RADIUS = 0.3;
constexpr double MIN_ANGLE = -0.6;
constexpr double MAX_ANGLE = 0.6;
}  // namespace

TEST_CASE("BicycleProjector construction stores model and limits")
{
  BicycleProjector projector(BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE);
  CHECK(projector.get_min_steering_angle_rad() == Approx(MIN_ANGLE));
  CHECK(projector.get_max_steering_angle_rad() == Approx(MAX_ANGLE));
  CHECK(projector.get_model().get_wheelbase_m() == Approx(WHEELBASE));
}

TEST_CASE("BicycleProjector step - zero rate freezes the steering angle")
{
  BicycleProjector projector(BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.2, 0.5, 0.0, 1.0);
  CHECK(result.steering_angle_rad == Approx(0.2));
}

TEST_CASE("BicycleProjector step - large rate reaches target in one step without overshoot")
{
  BicycleProjector projector(BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  // |delta| = 0.3, max_delta = rate * dt = 10.0 * 0.1 = 1.0 >> 0.3 → snaps exactly to target.
  auto result = projector.step(0.1, pose, 0.0, 0.3, 10.0, 1.0);
  CHECK(result.steering_angle_rad == Approx(0.3));
}

TEST_CASE("BicycleProjector step - rate-limited slew advances by rate*dt toward target")
{
  BicycleProjector projector(BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  // rate*dt = 0.05, target far away → expect 0.05 increment.
  auto result = projector.step(0.1, pose, 0.0, 0.5, 0.5, 1.0);
  CHECK(result.steering_angle_rad == Approx(0.05));
}

TEST_CASE("BicycleProjector step - negative steering rate is treated as magnitude")
{
  BicycleProjector projector(BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.0, 0.5, -0.5, 1.0);
  CHECK(result.steering_angle_rad == Approx(0.05));
}

TEST_CASE("BicycleProjector step - target above max saturates at max")
{
  BicycleProjector projector(BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  // With large rate the angle snaps to clamped_target == max.
  auto result = projector.step(0.1, pose, 0.0, 5.0, 100.0, 1.0);
  CHECK(result.steering_angle_rad == Approx(MAX_ANGLE));
}

TEST_CASE("BicycleProjector project - straight line traces +x with theta unchanged")
{
  BicycleProjector projector(BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  auto trajectory = projector.project(1.0, 0.1, pose, 0.0, 0.0, 1.0, 2.0);
  // ceil(1.0 / 0.1) + 1 = 11 samples
  CHECK(trajectory.size() == 11);
  CHECK(trajectory.front().time_s == Approx(0.0));
  CHECK(trajectory.front().pose.x == Approx(0.0));
  CHECK(trajectory.back().time_s == Approx(1.0));
  CHECK(trajectory.back().pose.x == Approx(2.0));  // x = v * t
  CHECK(trajectory.back().pose.y == Approx(0.0));
  CHECK(trajectory.back().pose.theta == Approx(0.0));
}

TEST_CASE("BicycleProjector project - ramp reaches target in expected number of steps")
{
  BicycleProjector projector(BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  // target=0.5, rate=0.5, dt=0.1 → step adds 0.05; takes 10 steps to reach 0.5.
  auto trajectory = projector.project(2.0, 0.1, pose, 0.0, 0.5, 0.5, 0.0);
  CHECK(trajectory[10].steering_angle_rad == Approx(0.5));
  CHECK(trajectory.back().steering_angle_rad == Approx(0.5));  // pinned to target afterwards
}

TEST_CASE("BicycleProjector project - sign symmetry: negating target mirrors trajectory across y=0")
{
  BicycleProjector projector(BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};

  auto left = projector.project(2.0, 0.05, pose, 0.0, 0.4, 1.0, 1.0);
  auto right = projector.project(2.0, 0.05, pose, 0.0, -0.4, 1.0, 1.0);
  REQUIRE(left.size() == right.size());

  for (std::size_t i = 0; i < left.size(); ++i) {
    CHECK(left[i].pose.x == Approx(right[i].pose.x));
    CHECK(left[i].pose.y == Approx(-right[i].pose.y));
    CHECK(left[i].pose.theta == Approx(-right[i].pose.theta));
    CHECK(left[i].steering_angle_rad == Approx(-right[i].steering_angle_rad));
  }
}

TEST_CASE("BicycleProjector project - initial state stored as element 0")
{
  BicycleProjector projector(BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{1.5, -2.0, 0.25};

  auto trajectory = projector.project(0.5, 0.1, pose, 0.1, 0.3, 0.5, 1.0);
  CHECK(trajectory.front().time_s == Approx(0.0));
  CHECK(trajectory.front().pose.x == Approx(1.5));
  CHECK(trajectory.front().pose.y == Approx(-2.0));
  CHECK(trajectory.front().pose.theta == Approx(0.25));
  CHECK(trajectory.front().steering_angle_rad == Approx(0.1));
}

TEST_CASE("BicycleProjector - no footprint set yields an empty footprint (never throws)")
{
  BicycleProjector projector(BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE);
  Pose2D pose{0.0, 0.0, 0.0};
  auto result = projector.step(0.1, pose, 0.0, 0.0, 0.0, 1.0);
  CHECK(result.footprint.empty());
}

TEST_CASE("BicycleProjector footprint - rectangle corners at a known pose")
{
  constexpr double FRONT = 3.0;
  constexpr double REAR = 1.0;
  constexpr double WIDTH = 2.0;
  BicycleProjector projector(
    BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS),
    MIN_ANGLE,
    MAX_ANGLE,
    AxleReference::REAR,
    rectangleFootprint(FRONT, REAR, WIDTH));

  // v=0, zero steering: pose stays at the origin with heading +x.
  Pose2D pose{0.0, 0.0, 0.0};
  auto result = projector.step(0.1, pose, 0.0, 0.0, 0.0, 0.0);
  REQUIRE(result.footprint.size() == 4);
  // Body frame corners: rear-right, front-right, front-left, rear-left.
  CHECK(result.footprint[0].x == Approx(-REAR));
  CHECK(result.footprint[0].y == Approx(-WIDTH / 2.0));
  CHECK(result.footprint[1].x == Approx(FRONT));
  CHECK(result.footprint[1].y == Approx(-WIDTH / 2.0));
  CHECK(result.footprint[2].x == Approx(FRONT));
  CHECK(result.footprint[2].y == Approx(WIDTH / 2.0));
  CHECK(result.footprint[3].x == Approx(-REAR));
  CHECK(result.footprint[3].y == Approx(WIDTH / 2.0));
}

TEST_CASE("BicycleProjector footprint - arbitrary polygon is carried through vertex for vertex")
{
  // A 5-vertex tapered nose, deliberately not a rectangle.
  const Footprint body{
    Point2D{-1.0, -0.9}, Point2D{2.0, -0.9}, Point2D{2.8, 0.0}, Point2D{2.0, 0.9}, Point2D{-1.0, 0.9}};
  BicycleProjector projector(
    BicycleModel(WHEELBASE, TRACK, WHEEL_RADIUS), MIN_ANGLE, MAX_ANGLE, AxleReference::REAR, body);

  Pose2D pose{0.0, 0.0, 0.0};
  auto result = projector.step(0.1, pose, 0.0, 0.0, 0.0, 0.0);
  REQUIRE(result.footprint.size() == body.size());
  for (std::size_t i = 0; i < body.size(); ++i) {
    CHECK(result.footprint[i].x == Approx(body[i].x));
    CHECK(result.footprint[i].y == Approx(body[i].y));
  }
}

TEST_CASE("BicycleProjector - FRONT reference offsets the pose by a wheelbase")
{
  const BicycleModel model(WHEELBASE, TRACK, WHEEL_RADIUS);
  BicycleProjector rear(model, MIN_ANGLE, MAX_ANGLE, AxleReference::REAR);
  BicycleProjector front(model, MIN_ANGLE, MAX_ANGLE, AxleReference::FRONT);

  // Straight ahead from the origin: the front-axle pose leads the rear-axle pose by the wheelbase.
  auto rear_traj = rear.project(1.0, 0.1, Pose2D{0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 1.0);
  auto front_traj = front.project(1.0, 0.1, Pose2D{WHEELBASE, 0.0, 0.0}, 0.0, 0.0, 0.0, 1.0);
  REQUIRE(rear_traj.size() == front_traj.size());
  for (std::size_t i = 0; i < rear_traj.size(); ++i) {
    CHECK(front_traj[i].pose.x == Approx(rear_traj[i].pose.x + WHEELBASE));
    CHECK(front_traj[i].pose.y == Approx(rear_traj[i].pose.y));
    CHECK(front_traj[i].pose.theta == Approx(rear_traj[i].pose.theta));
  }
}

TEST_CASE("BicycleProjector - FRONT reference traces the same curve as REAR, offset forward")
{
  const BicycleModel model(WHEELBASE, TRACK, WHEEL_RADIUS);
  const double steering = 0.3;
  BicycleProjector rear(model, MIN_ANGLE, MAX_ANGLE, AxleReference::REAR);
  BicycleProjector front(model, MIN_ANGLE, MAX_ANGLE, AxleReference::FRONT);

  // Same physical vehicle, same turn: seed each with its own axle's start pose.
  auto rear_traj = rear.project(2.0, 0.05, Pose2D{0.0, 0.0, 0.0}, steering, steering, 0.0, 1.5);
  auto front_traj = front.project(2.0, 0.05, Pose2D{WHEELBASE, 0.0, 0.0}, steering, steering, 0.0, 1.5);
  REQUIRE(rear_traj.size() == front_traj.size());
  for (std::size_t i = 0; i < rear_traj.size(); ++i) {
    // Headings match (one rigid chassis) and the front axle sits a wheelbase ahead along it.
    const double theta = rear_traj[i].pose.theta;
    CHECK(front_traj[i].pose.theta == Approx(theta));
    CHECK(front_traj[i].pose.x == Approx(rear_traj[i].pose.x + WHEELBASE * std::cos(theta)));
    CHECK(front_traj[i].pose.y == Approx(rear_traj[i].pose.y + WHEELBASE * std::sin(theta)));
  }
}

TEST_CASE("BicycleProjector - footprint is anchored at whichever axle is the reference")
{
  const BicycleModel model(WHEELBASE, TRACK, WHEEL_RADIUS);
  // Same physical body described from each axle: bumper 0.5 m past the front axle, 1.0 m behind
  // the rear axle. From the rear axle that is (WHEELBASE + 0.5) forward; from the front axle it is
  // 0.5 forward and (WHEELBASE + 1.0) back.
  BicycleProjector rear(
    model, MIN_ANGLE, MAX_ANGLE, AxleReference::REAR, rectangleFootprint(WHEELBASE + 0.5, 1.0, 2.0));
  BicycleProjector front(
    model, MIN_ANGLE, MAX_ANGLE, AxleReference::FRONT, rectangleFootprint(0.5, WHEELBASE + 1.0, 2.0));

  auto rear_state = rear.step(0.1, Pose2D{0.0, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0);
  auto front_state = front.step(0.1, Pose2D{WHEELBASE, 0.0, 0.0}, 0.0, 0.0, 0.0, 0.0);
  REQUIRE(rear_state.footprint.size() == 4);
  REQUIRE(front_state.footprint.size() == 4);
  // Both descriptions must land the body in the same place in the world.
  for (std::size_t i = 0; i < 4; ++i) {
    CHECK(front_state.footprint[i].x == Approx(rear_state.footprint[i].x));
    CHECK(front_state.footprint[i].y == Approx(rear_state.footprint[i].y));
  }
}

}  // namespace polymath::kinematics
