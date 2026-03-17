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

#include "polymath_kinematics/bicycle_model.hpp"

#include <cmath>
#include <limits>

namespace polymath::kinematics
{

BicycleBodyVelocity BicycleModel::steeringToBodyVelocity(double velocity, double steering_angle)
{
  double angular_velocity = velocity * std::tan(steering_angle) / wheelbase_m_;

  return BicycleBodyVelocity{velocity, angular_velocity};
}

BicycleSteeringState BicycleModel::bodyVelocityToSteering(double linear_velocity, double angular_velocity)
{
  if (std::abs(linear_velocity) < ZERO_VELOCITY_THRESHOLD) {
    // Stationary: cannot determine steering angle from the bicycle model
    return BicycleSteeringState{linear_velocity, 0.0, std::numeric_limits<double>::infinity(), 0.0, 0.0, 0.0, 0.0};
  }

  if (std::abs(angular_velocity) < ZERO_VELOCITY_THRESHOLD) {
    // Straight line: no rotation, all wheels spin at the same rate
    double wheel_rad_s = linear_velocity / wheel_radius_m_;
    return BicycleSteeringState{
      linear_velocity,
      0.0,
      std::numeric_limits<double>::infinity(),
      wheel_rad_s,
      wheel_rad_s,
      wheel_rad_s,
      wheel_rad_s};
  }

  // steering_angle = atan(omega * L / v) inverts the forward kinematic
  // relation omega = v * tan(steering_angle) / L.
  // atan (not atan2) is intentional: the result must stay in (-pi/2, pi/2),
  // which is the valid physical range for a steering angle. atan2 would add
  // +/-pi when v < 0, placing the result in the wrong quadrant for reverse.
  double steering_angle = std::atan(angular_velocity * wheelbase_m_ / linear_velocity);
  double turning_radius = turningRadius(steering_angle);
  double half_track = track_width_m_ / 2.0;

  // Signed lateral distance from ICR to each rear wheel along the rear axle.
  // ICR lies on the rear axle line, so the distance is purely lateral.
  // Positive = wheel is on the outer side of the turn.
  double rear_left_r = turning_radius - half_track;
  double rear_right_r = turning_radius + half_track;

  // Signed distance from ICR to each front wheel. The front wheels are offset
  // by the wheelbase longitudinally, so the distance is the hypotenuse of
  // (lateral_offset, wheelbase). copysign preserves the sign of the lateral
  // offset so that each wheel's speed has the correct direction — this matters
  // when the ICR falls between the rear wheels (|R| < half_track), where the
  // inner and outer wheels rotate in opposite directions.
  double front_left_r = std::copysign(std::hypot(rear_left_r, wheelbase_m_), rear_left_r);
  double front_right_r = std::copysign(std::hypot(rear_right_r, wheelbase_m_), rear_right_r);

  // Each wheel's ground speed = omega * distance_to_ICR (rigid body kinematics).
  // Convert to wheel angular velocity by dividing by wheel radius.
  double rl = angular_velocity * rear_left_r / wheel_radius_m_;
  double rr = angular_velocity * rear_right_r / wheel_radius_m_;
  double fl = angular_velocity * front_left_r / wheel_radius_m_;
  double fr = angular_velocity * front_right_r / wheel_radius_m_;

  return BicycleSteeringState{linear_velocity, steering_angle, turning_radius, fr, fl, rr, rl};
}

double BicycleModel::turningRadius(double steering_angle)
{
  double tan_steering = std::tan(steering_angle);

  if (std::abs(tan_steering) < ZERO_VELOCITY_THRESHOLD) {
    return std::numeric_limits<double>::infinity();
  }

  return wheelbase_m_ / tan_steering;
}

double BicycleModel::steeringAngleFromRadius(double radius)
{
  if (std::isinf(radius)) {
    return 0.0;
  }

  return std::atan(wheelbase_m_ / radius);
}

}  // namespace polymath::kinematics
