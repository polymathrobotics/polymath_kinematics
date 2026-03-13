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

#ifndef POLYMATH_KINEMATICS__BICYCLE_MODEL_HPP__
#define POLYMATH_KINEMATICS__BICYCLE_MODEL_HPP__

namespace polymath::kinematics
{

struct BicycleSteeringState
{
  double velocity_m_s;
  double steering_angle_rad;
  double turning_radius_m;
  double front_right_wheel_rad_s;
  double front_left_wheel_rad_s;
  double rear_right_wheel_rad_s;
  double rear_left_wheel_rad_s;
};

struct BicycleBodyVelocity
{
  double linear_velocity_m_s;
  double angular_velocity_rad_s;
};

class BicycleModel
{
public:
  BicycleModel(double wheelbase_m, double track_width_m, double wheel_radius_m)
  : wheelbase_m_(wheelbase_m)
  , track_width_m_(track_width_m)
  , wheel_radius_m_(wheel_radius_m)
  {}

  ~BicycleModel() = default;

  /// @brief Convert velocity and steering angle to body velocities
  /// @param velocity Forward velocity in m/s
  /// @param steering_angle Steering angle in radians
  /// @return Linear and angular body velocities
  BicycleBodyVelocity steeringToBodyVelocity(double velocity, double steering_angle);

  /// @brief Convert body velocities to steering angle
  /// @param linear_velocity Forward velocity in m/s
  /// @param angular_velocity Angular velocity in rad/s
  /// @return Velocity and required steering angle
  BicycleSteeringState bodyVelocityToSteering(double linear_velocity, double angular_velocity);

  /// @brief Calculate turning radius from steering angle
  /// @param steering_angle Steering angle in radians
  /// @return Turning radius in meters (infinity for straight line)
  double turningRadius(double steering_angle);

  /// @brief Calculate steering angle from desired turning radius
  /// @param radius Turning radius in meters
  /// @return Required steering angle in radians
  double steeringAngleFromRadius(double radius);

  double get_wheelbase_m() const
  {
    return wheelbase_m_;
  }

  double get_track_width_m() const
  {
    return track_width_m_;
  }

  double get_wheel_radius_m() const
  {
    return wheel_radius_m_;
  }

private:
  double wheelbase_m_;
  double track_width_m_;
  double wheel_radius_m_;

  /// @brief Threshold below which a velocity is treated as zero to avoid numerical
  /// singularities (e.g. division by zero in steering angle and turning radius calculations)
  static constexpr double ZERO_VELOCITY_THRESHOLD = 1e-9;
};

}  // namespace polymath::kinematics

#endif  // POLYMATH_KINEMATICS__BICYCLE_MODEL_HPP__
