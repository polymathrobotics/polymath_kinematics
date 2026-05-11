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
constexpr double kWheelbase = 2.5;
constexpr double kTrack = 1.5;
constexpr double kWheelRadius = 0.3;
constexpr double kMinAngle = -0.6;
constexpr double kMaxAngle = 0.6;
}  // namespace

TEST_CASE("BicycleProjector construction stores model and limits")
{
  BicycleProjector projector(BicycleModel(kWheelbase, kTrack, kWheelRadius), kMinAngle, kMaxAngle);
  CHECK(projector.get_min_steering_angle_rad() == Approx(kMinAngle));
  CHECK(projector.get_max_steering_angle_rad() == Approx(kMaxAngle));
  CHECK(projector.get_model().get_wheelbase_m() == Approx(kWheelbase));
}

TEST_CASE("BicycleProjector step - zero rate freezes the steering angle")
{
  BicycleProjector projector(BicycleModel(kWheelbase, kTrack, kWheelRadius), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.2, 0.5, 0.0, 1.0);
  CHECK(result.steering_angle_rad == Approx(0.2));
}

TEST_CASE("BicycleProjector step - large rate reaches target in one step without overshoot")
{
  BicycleProjector projector(BicycleModel(kWheelbase, kTrack, kWheelRadius), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  // |delta| = 0.3, max_delta = rate * dt = 10.0 * 0.1 = 1.0 >> 0.3 → snaps exactly to target.
  auto result = projector.step(0.1, pose, 0.0, 0.3, 10.0, 1.0);
  CHECK(result.steering_angle_rad == Approx(0.3));
}

TEST_CASE("BicycleProjector step - rate-limited slew advances by rate*dt toward target")
{
  BicycleProjector projector(BicycleModel(kWheelbase, kTrack, kWheelRadius), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  // rate*dt = 0.05, target far away → expect 0.05 increment.
  auto result = projector.step(0.1, pose, 0.0, 0.5, 0.5, 1.0);
  CHECK(result.steering_angle_rad == Approx(0.05));
}

TEST_CASE("BicycleProjector step - negative steering rate is treated as magnitude")
{
  BicycleProjector projector(BicycleModel(kWheelbase, kTrack, kWheelRadius), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  auto result = projector.step(0.1, pose, 0.0, 0.5, -0.5, 1.0);
  CHECK(result.steering_angle_rad == Approx(0.05));
}

TEST_CASE("BicycleProjector step - target above max saturates at max")
{
  BicycleProjector projector(BicycleModel(kWheelbase, kTrack, kWheelRadius), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  // With large rate the angle snaps to clamped_target == max.
  auto result = projector.step(0.1, pose, 0.0, 5.0, 100.0, 1.0);
  CHECK(result.steering_angle_rad == Approx(kMaxAngle));
}

TEST_CASE("BicycleProjector project - straight line traces +x with theta unchanged")
{
  BicycleProjector projector(BicycleModel(kWheelbase, kTrack, kWheelRadius), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  auto trajectory = projector.project(1.0, 0.1, pose, 0.0, 0.0, 1.0, 2.0);
  // ceil(1.0 / 0.1) + 1 = 11 samples
  CHECK_EQ(trajectory.size(), 11);
  CHECK_EQ(trajectory.front().time_s, Approx(0.0));
  CHECK_EQ(trajectory.front().pose.x, Approx(0.0));
  CHECK_EQ(trajectory.back().time_s, Approx(1.0));
  CHECK_EQ(trajectory.back().pose.x, Approx(2.0));  // x = v * t
  CHECK_EQ(trajectory.back().pose.y, Approx(0.0));
  CHECK_EQ(trajectory.back().pose.theta, Approx(0.0));
}

TEST_CASE("BicycleProjector project - ramp reaches target in expected number of steps")
{
  BicycleProjector projector(BicycleModel(kWheelbase, kTrack, kWheelRadius), kMinAngle, kMaxAngle);
  Pose2D pose{0.0, 0.0, 0.0};

  // target=0.5, rate=0.5, dt=0.1 → step adds 0.05; takes 10 steps to reach 0.5.
  auto trajectory = projector.project(2.0, 0.1, pose, 0.0, 0.5, 0.5, 0.0);
  CHECK(trajectory[10].steering_angle_rad == Approx(0.5));
  CHECK(trajectory.back().steering_angle_rad == Approx(0.5));  // pinned to target afterwards
}

TEST_CASE("BicycleProjector project - sign symmetry: negating target mirrors trajectory across y=0")
{
  BicycleProjector projector(BicycleModel(kWheelbase, kTrack, kWheelRadius), kMinAngle, kMaxAngle);
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
  BicycleProjector projector(BicycleModel(kWheelbase, kTrack, kWheelRadius), kMinAngle, kMaxAngle);
  Pose2D pose{1.5, -2.0, 0.25};

  auto trajectory = projector.project(0.5, 0.1, pose, 0.1, 0.3, 0.5, 1.0);
  CHECK(trajectory.front().time_s == Approx(0.0));
  CHECK(trajectory.front().pose.x == Approx(1.5));
  CHECK(trajectory.front().pose.y == Approx(-2.0));
  CHECK(trajectory.front().pose.theta == Approx(0.25));
  CHECK(trajectory.front().steering_angle_rad == Approx(0.1));
}

}  // namespace polymath::kinematics
