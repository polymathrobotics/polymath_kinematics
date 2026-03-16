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

#ifndef POLYMATH_KINEMATICS__DIFFERENTIAL_DRIVE_MODEL_HPP__
#define POLYMATH_KINEMATICS__DIFFERENTIAL_DRIVE_MODEL_HPP__

namespace polymath::kinematics
{

struct DifferentialDriveWheelVelocities
{
  double left_wheel_velocity_rad_s;
  double right_wheel_velocity_rad_s;
};

struct DifferentialDriveBodyVelocity
{
  double linear_velocity_m_s;
  double angular_velocity_rad_s;
};

class DifferentialDriveModel
{
public:
  DifferentialDriveModel(double wheel_radius_m, double track_width_m)
  : wheel_radius_m_(wheel_radius_m)
  , track_width_m_(track_width_m)
  {}

  ~DifferentialDriveModel() = default;

  /// @brief Convert wheel velocities to body velocities
  /// @param left_wheel_vel Left wheel angular velocity in rad/s
  /// @param right_wheel_vel Right wheel angular velocity in rad/s
  /// @return Body linear and angular velocities
  DifferentialDriveBodyVelocity wheelVelocitiesToBodyVelocity(double left_wheel_vel, double right_wheel_vel);

  /// @brief Convert body velocities to wheel velocities
  /// @param linear_vel Desired linear velocity in m/s
  /// @param angular_vel Desired angular velocity in rad/s
  /// @return Required wheel angular velocities
  DifferentialDriveWheelVelocities bodyVelocityToWheelVelocities(double linear_vel, double angular_vel);

  double get_wheel_radius_m() const
  {
    return wheel_radius_m_;
  }

  double get_track_width_m() const
  {
    return track_width_m_;
  }

private:
  double wheel_radius_m_;
  double track_width_m_;
};

}  // namespace polymath::kinematics

#endif  // POLYMATH_KINEMATICS__DIFFERENTIAL_DRIVE_MODEL_HPP__
