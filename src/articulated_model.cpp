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

#include "polymath_kinematics/articulated_model.hpp"

#include <cmath>
#include <limits>

namespace polymath::kinematics
{

constexpr inline double square(double x)
{
  return x * x;
}

ArticulatedVehicleState ArticulatedModel::bodyVelocityToVehicleState(
  double linear_velocity_m_s, double angular_velocity_rad_s)
{
  // Guard: fully stationary — sqrt denominator collapses to 0, producing 0/0 = NaN
  if (
    std::abs(linear_velocity_m_s) < ZERO_VELOCITY_THRESHOLD &&
    std::abs(angular_velocity_rad_s) < ZERO_VELOCITY_THRESHOLD)
  {
    return ArticulatedVehicleState{
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
  }

  // Guard: straight line — articulation_angle = 0, so sin(0) = 0 → radii = inf → inf * 0 = NaN wheel speeds
  if (std::abs(angular_velocity_rad_s) < ZERO_VELOCITY_THRESHOLD) {
    double front_wheel_rad_s = linear_velocity_m_s / front_wheel_radius_m_;
    double rear_wheel_rad_s = linear_velocity_m_s / rear_wheel_radius_m_;
    return ArticulatedVehicleState{
      0.0,
      linear_velocity_m_s,
      front_wheel_rad_s,
      front_wheel_rad_s,
      rear_wheel_rad_s,
      rear_wheel_rad_s,
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
  }

  // copysign selects + for forward, - for reverse.
  double acos_calculation = std::acos(
    articulation_to_rear_axle_m_ * (articulation_turning_velocity_rad_s_ - angular_velocity_rad_s) /
    std::hypot(angular_velocity_rad_s * articulation_to_front_axle_m_, linear_velocity_m_s));

  double atan2_calculation = std::atan2(linear_velocity_m_s, angular_velocity_rad_s * articulation_to_front_axle_m_);
  double articulation_angle = std::copysign(acos_calculation, linear_velocity_m_s) - atan2_calculation;

  double cos_articulation = std::cos(articulation_angle);
  double sin_articulation = std::sin(articulation_angle);

  double front_radius =
    (articulation_to_front_axle_m_ * cos_articulation + articulation_to_rear_axle_m_) / sin_articulation;
  double rear_radius =
    (articulation_to_rear_axle_m_ * cos_articulation + articulation_to_front_axle_m_) / sin_articulation;

  double front_right_wheel_speed =
    (front_radius + front_track_width_m_ / 2.0) * angular_velocity_rad_s / front_wheel_radius_m_;
  double front_left_wheel_speed =
    (front_radius - front_track_width_m_ / 2.0) * angular_velocity_rad_s / front_wheel_radius_m_;
  double rear_right_wheel_speed =
    (rear_radius + rear_track_width_m_ / 2.0) * angular_velocity_rad_s / rear_wheel_radius_m_;
  double rear_left_wheel_speed =
    (rear_radius - rear_track_width_m_ / 2.0) * angular_velocity_rad_s / rear_wheel_radius_m_;

  return ArticulatedVehicleState{
    articulation_angle,
    linear_velocity_m_s,
    front_right_wheel_speed,
    front_left_wheel_speed,
    rear_right_wheel_speed,
    rear_left_wheel_speed,
    front_radius,
    rear_radius};
}

ArticulatedAxleVelocities ArticulatedModel::articulationToAxleVelocities(
  double linear_velocity_m_s, double articulation_angle_rad)
{
  double front_axle_turning_velocity =
    (linear_velocity_m_s * std::sin(articulation_angle_rad) +
     articulation_to_rear_axle_m_ * articulation_turning_velocity_rad_s_) /
    (articulation_to_front_axle_m_ * std::cos(articulation_angle_rad) + articulation_to_rear_axle_m_);

  double rear_axle_turning_velocity = front_axle_turning_velocity - articulation_turning_velocity_rad_s_;

  return ArticulatedAxleVelocities{linear_velocity_m_s, front_axle_turning_velocity, rear_axle_turning_velocity};
}

}  // namespace polymath::kinematics
