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
#include "polymath_kinematics/bicycle_model.hpp"

namespace polymath::kinematics
{

TEST_CASE("BicycleModel construction")
{
  BicycleModel model(2.5, 1.5, 0.3);
  CHECK(model.get_wheelbase_m() == 2.5);
  CHECK(model.get_track_width_m() == 1.5);
  CHECK(model.get_wheel_radius_m() == 0.3);
}

TEST_CASE("BicycleModel steeringToBodyVelocity - straight line")
{
  BicycleModel model(2.5, 1.5, 0.3);

  auto result = model.steeringToBodyVelocity(5.0, 0.0);

  CHECK(result.linear_velocity_m_s == Approx(5.0));
  CHECK(result.angular_velocity_rad_s == Approx(0.0));
}

TEST_CASE("BicycleModel steeringToBodyVelocity - turning")
{
  BicycleModel model(2.5, 1.5, 0.3);

  double steering_angle = M_PI / 4;  // 45 degrees
  auto result = model.steeringToBodyVelocity(5.0, steering_angle);

  CHECK(result.linear_velocity_m_s == Approx(5.0));
  // angular_velocity = 5.0 * tan(45) / 2.5 = 5.0 * 1.0 / 2.5 = 2.0 rad/s
  CHECK(result.angular_velocity_rad_s == Approx(2.0));
}

TEST_CASE("BicycleModel steeringToBodyVelocity - right turn")
{
  BicycleModel model(2.5, 1.5, 0.3);

  double steering_angle = -M_PI / 4;  // -45 degrees (right turn)
  auto result = model.steeringToBodyVelocity(5.0, steering_angle);

  CHECK(result.linear_velocity_m_s == Approx(5.0));
  CHECK(result.angular_velocity_rad_s == Approx(-2.0));
}

TEST_CASE("BicycleModel bodyVelocityToSteering - straight line")
{
  BicycleModel model(2.5, 1.5, 0.3);

  auto result = model.bodyVelocityToSteering(5.0, 0.0);

  CHECK(result.velocity_m_s == Approx(5.0));
  CHECK(result.steering_angle_rad == Approx(0.0));
  CHECK(std::isinf(result.turning_radius_m));

  // All wheels should have the same angular velocity for straight line
  double expected_wheel_vel = 5.0 / 0.3;  // linear_vel / wheel_radius
  CHECK(result.front_left_wheel_rad_s == Approx(expected_wheel_vel));
  CHECK(result.front_right_wheel_rad_s == Approx(expected_wheel_vel));
  CHECK(result.rear_left_wheel_rad_s == Approx(expected_wheel_vel));
  CHECK(result.rear_right_wheel_rad_s == Approx(expected_wheel_vel));
}

TEST_CASE("BicycleModel bodyVelocityToSteering - turning")
{
  BicycleModel model(2.5, 1.5, 0.3);

  // angular_velocity = 2.0 rad/s, linear_velocity = 5.0 m/s
  // steering_angle = atan(2.0 * 2.5 / 5.0) = atan(1.0) = pi/4
  auto result = model.bodyVelocityToSteering(5.0, 2.0);

  CHECK(result.velocity_m_s == Approx(5.0));
  CHECK(result.steering_angle_rad == Approx(M_PI / 4));

  // turning_radius = wheelbase / tan(steering_angle) = 2.5 / 1.0 = 2.5
  CHECK(result.turning_radius_m == Approx(2.5));

  // For left turn (positive angular velocity), inner wheels (left) should be slower
  CHECK(result.rear_left_wheel_rad_s < result.rear_right_wheel_rad_s);
  CHECK(result.front_left_wheel_rad_s < result.front_right_wheel_rad_s);
}

TEST_CASE("BicycleModel bodyVelocityToSteering - zero linear velocity")
{
  BicycleModel model(2.5, 1.5, 0.3);

  // Zero linear velocity with nonzero angular: degenerate for bicycle model
  auto result = model.bodyVelocityToSteering(0.0, 1.0);

  CHECK(result.velocity_m_s == Approx(0.0));
  CHECK(result.steering_angle_rad == Approx(0.0));
  CHECK(std::isinf(result.turning_radius_m));
  CHECK(result.front_left_wheel_rad_s == Approx(0.0));
  CHECK(result.front_right_wheel_rad_s == Approx(0.0));
  CHECK(result.rear_left_wheel_rad_s == Approx(0.0));
  CHECK(result.rear_right_wheel_rad_s == Approx(0.0));
}

TEST_CASE("BicycleModel bodyVelocityToSteering - stationary")
{
  BicycleModel model(2.5, 1.5, 0.3);

  // Both inputs zero: fully stationary
  auto result = model.bodyVelocityToSteering(0.0, 0.0);

  CHECK(result.velocity_m_s == Approx(0.0));
  CHECK(result.steering_angle_rad == Approx(0.0));
  CHECK(std::isinf(result.turning_radius_m));
  CHECK(result.front_left_wheel_rad_s == Approx(0.0));
  CHECK(result.front_right_wheel_rad_s == Approx(0.0));
  CHECK(result.rear_left_wheel_rad_s == Approx(0.0));
  CHECK(result.rear_right_wheel_rad_s == Approx(0.0));
}

TEST_CASE("BicycleModel turningRadius")
{
  BicycleModel model(2.5, 1.5, 0.3);

  // Straight line
  CHECK(std::isinf(model.turningRadius(0.0)));

  // 45 degree turn: radius = wheelbase / tan(pi/4) = 2.5 / 1.0 = 2.5
  CHECK(model.turningRadius(M_PI / 4) == Approx(2.5));
}

TEST_CASE("BicycleModel steeringAngleFromRadius")
{
  BicycleModel model(2.5, 1.5, 0.3);

  // Infinite radius -> straight line
  CHECK(model.steeringAngleFromRadius(std::numeric_limits<double>::infinity()) == Approx(0.0));

  // radius = wheelbase -> 45 degree steering
  CHECK(model.steeringAngleFromRadius(2.5) == Approx(M_PI / 4));
}

TEST_CASE("BicycleModel roundtrip")
{
  BicycleModel model(2.5, 1.5, 0.3);

  double velocity = 3.0;
  double steering_angle = 0.3;

  auto body_vel = model.steeringToBodyVelocity(velocity, steering_angle);
  auto steering = model.bodyVelocityToSteering(body_vel.linear_velocity_m_s, body_vel.angular_velocity_rad_s);

  CHECK(steering.velocity_m_s == Approx(velocity));
  CHECK(steering.steering_angle_rad == Approx(steering_angle));
}

TEST_CASE("BicycleModel turningRadius/steeringAngleFromRadius roundtrip")
{
  BicycleModel model(2.5, 1.5, 0.3);

  double steering_angle = 0.5;

  double radius = model.turningRadius(steering_angle);
  double recovered_angle = model.steeringAngleFromRadius(radius);

  CHECK(recovered_angle == Approx(steering_angle));
}

TEST_CASE("BicycleModel steeringAngleFromRadius - negative radius")
{
  BicycleModel model(2.5, 1.5, 0.3);

  // Negative radius (right turn) should give negative steering angle
  // radius = -2.5 -> steering = atan(2.5 / -2.5) = atan(-1) = -pi/4
  CHECK(model.steeringAngleFromRadius(-2.5) == Approx(-M_PI / 4));
}

TEST_CASE("BicycleModel turningRadius/steeringAngleFromRadius roundtrip - negative radius")
{
  BicycleModel model(2.5, 1.5, 0.3);

  double steering_angle = -0.5;  // Right turn

  double radius = model.turningRadius(steering_angle);
  CHECK(radius < 0.0);  // Negative radius for right turn

  double recovered_angle = model.steeringAngleFromRadius(radius);
  CHECK(recovered_angle == Approx(steering_angle));
}

TEST_CASE("BicycleModel bodyVelocityToSteering - right turn")
{
  BicycleModel model(2.5, 1.5, 0.3);

  // Negative angular velocity -> right turn
  auto result = model.bodyVelocityToSteering(5.0, -2.0);

  CHECK(result.velocity_m_s == Approx(5.0));
  CHECK(result.steering_angle_rad == Approx(-M_PI / 4));
  CHECK(result.turning_radius_m == Approx(-2.5));

  // For right turn with forward motion, inner wheels (right) should be slower
  CHECK(result.rear_right_wheel_rad_s < result.rear_left_wheel_rad_s);
  CHECK(result.front_right_wheel_rad_s < result.front_left_wheel_rad_s);

  // All wheel speeds should be positive (vehicle moving forward)
  CHECK(result.rear_left_wheel_rad_s > 0.0);
  CHECK(result.rear_right_wheel_rad_s > 0.0);
  CHECK(result.front_left_wheel_rad_s > 0.0);
  CHECK(result.front_right_wheel_rad_s > 0.0);
}

TEST_CASE("BicycleModel bodyVelocityToSteering - wheel speed values")
{
  BicycleModel model(2.5, 1.5, 0.3);

  double v = 5.0;
  double omega = 2.0;

  auto result = model.bodyVelocityToSteering(v, omega);

  double R = result.turning_radius_m;  // 2.5
  double half_track = 0.75;

  // Rear wheels: omega * (R +/- half_track) / wheel_radius
  CHECK(result.rear_left_wheel_rad_s == Approx(omega * (R - half_track) / 0.3));
  CHECK(result.rear_right_wheel_rad_s == Approx(omega * (R + half_track) / 0.3));

  // Front wheels: omega * hypot(R +/- half_track, wheelbase) / wheel_radius
  double front_left_dist = std::hypot(R - half_track, 2.5);
  double front_right_dist = std::hypot(R + half_track, 2.5);
  CHECK(result.front_left_wheel_rad_s == Approx(omega * front_left_dist / 0.3));
  CHECK(result.front_right_wheel_rad_s == Approx(omega * front_right_dist / 0.3));
}

TEST_CASE("BicycleModel bodyVelocityToSteering - reverse straight")
{
  BicycleModel model(2.5, 1.5, 0.3);

  auto result = model.bodyVelocityToSteering(-5.0, 0.0);

  CHECK(result.velocity_m_s == Approx(-5.0));
  CHECK(result.steering_angle_rad == Approx(0.0));

  // All wheel speeds should be negative (backward rolling)
  double expected_wheel_vel = -5.0 / 0.3;
  CHECK(result.front_left_wheel_rad_s == Approx(expected_wheel_vel));
  CHECK(result.front_right_wheel_rad_s == Approx(expected_wheel_vel));
  CHECK(result.rear_left_wheel_rad_s == Approx(expected_wheel_vel));
  CHECK(result.rear_right_wheel_rad_s == Approx(expected_wheel_vel));
}

TEST_CASE("BicycleModel roundtrip - negative steering angle")
{
  BicycleModel model(2.5, 1.5, 0.3);

  double velocity = 3.0;
  double steering_angle = -0.3;  // Right turn

  auto body_vel = model.steeringToBodyVelocity(velocity, steering_angle);
  auto steering = model.bodyVelocityToSteering(body_vel.linear_velocity_m_s, body_vel.angular_velocity_rad_s);

  CHECK(steering.velocity_m_s == Approx(velocity));
  CHECK(steering.steering_angle_rad == Approx(steering_angle));
}

TEST_CASE("BicycleModel bodyVelocityToSteering - reverse left turn")
{
  BicycleModel model(2.5, 1.5, 0.3);

  // Reversing (v < 0) + CCW body rotation (omega > 0): front wheels must steer RIGHT (delta < 0)
  // delta = atan(2.0 * 2.5 / -5.0) = atan(-1) = -pi/4
  auto result = model.bodyVelocityToSteering(-5.0, 2.0);

  CHECK(result.velocity_m_s == Approx(-5.0));
  CHECK(result.steering_angle_rad == Approx(-M_PI / 4));
  CHECK(result.turning_radius_m == Approx(-2.5));

  // All wheels move backward
  CHECK(result.rear_left_wheel_rad_s < 0.0);
  CHECK(result.rear_right_wheel_rad_s < 0.0);
  CHECK(result.front_left_wheel_rad_s < 0.0);
  CHECK(result.front_right_wheel_rad_s < 0.0);
}

TEST_CASE("BicycleModel bodyVelocityToSteering - reverse right turn")
{
  BicycleModel model(2.5, 1.5, 0.3);

  // Reversing (v < 0) + CW body rotation (omega < 0): front wheels must steer LEFT (delta > 0)
  // delta = atan(-2.0 * 2.5 / -5.0) = atan(1) = pi/4
  auto result = model.bodyVelocityToSteering(-5.0, -2.0);

  CHECK(result.velocity_m_s == Approx(-5.0));
  CHECK(result.steering_angle_rad == Approx(M_PI / 4));
  CHECK(result.turning_radius_m == Approx(2.5));

  // All wheels move backward
  CHECK(result.rear_left_wheel_rad_s < 0.0);
  CHECK(result.rear_right_wheel_rad_s < 0.0);
  CHECK(result.front_left_wheel_rad_s < 0.0);
  CHECK(result.front_right_wheel_rad_s < 0.0);
}

TEST_CASE("BicycleModel roundtrip - reverse velocity")
{
  BicycleModel model(2.5, 1.5, 0.3);

  double velocity = -3.0;
  double steering_angle = -0.3;  // Right steer while reversing

  auto body_vel = model.steeringToBodyVelocity(velocity, steering_angle);
  auto steering = model.bodyVelocityToSteering(body_vel.linear_velocity_m_s, body_vel.angular_velocity_rad_s);

  CHECK(steering.velocity_m_s == Approx(velocity));
  CHECK(steering.steering_angle_rad == Approx(steering_angle));
}

}  // namespace polymath::kinematics
