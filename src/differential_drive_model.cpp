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

#include "polymath_kinematics/differential_drive_model.hpp"

namespace polymath::kinematics
{

DifferentialDriveBodyVelocity DifferentialDriveModel::wheelVelocitiesToBodyVelocity(
  double left_wheel_vel, double right_wheel_vel)
{
  // Convert wheel angular velocities to linear velocities at the wheel
  double left_linear = left_wheel_vel * wheel_radius_m_;
  double right_linear = right_wheel_vel * wheel_radius_m_;

  // Body linear velocity is the average of the two wheel linear velocities
  double linear_velocity = (left_linear + right_linear) / 2.0;

  // Body angular velocity is the difference divided by track width
  double angular_velocity = (right_linear - left_linear) / track_width_m_;

  return DifferentialDriveBodyVelocity{linear_velocity, angular_velocity};
}

DifferentialDriveWheelVelocities DifferentialDriveModel::bodyVelocityToWheelVelocities(
  double linear_vel, double angular_vel)
{
  // Calculate the linear velocity at each wheel
  double left_linear = linear_vel - angular_vel * (track_width_m_ / 2.0);
  double right_linear = linear_vel + angular_vel * (track_width_m_ / 2.0);

  // Convert linear velocities to angular velocities
  double left_wheel_vel = left_linear / wheel_radius_m_;
  double right_wheel_vel = right_linear / wheel_radius_m_;

  return DifferentialDriveWheelVelocities{left_wheel_vel, right_wheel_vel};
}

}  // namespace polymath::kinematics
