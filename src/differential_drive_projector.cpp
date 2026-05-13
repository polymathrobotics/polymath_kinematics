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

#include "polymath_kinematics/differential_drive_projector.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace polymath::kinematics
{

namespace
{

/// @brief Ramp `current` toward `clamped_target` by at most |rate| * dt, never overshooting.
double rampedAdvance(double current, double clamped_target, double rate, double dt_s)
{
  double max_delta = std::abs(rate) * dt_s;
  double delta = clamped_target - current;
  if (std::abs(delta) > max_delta) {
    delta = std::copysign(max_delta, delta);
  }
  return current + delta;
}

}  // namespace

DifferentialDriveProjectedState DifferentialDriveProjector::step(
  double dt_s,
  const Pose2D & current_pose,
  double current_linear_velocity_m_s,
  double current_angular_velocity_rad_s,
  double target_linear_velocity_m_s,
  double target_angular_velocity_rad_s,
  double linear_acceleration_m_s2,
  double angular_acceleration_rad_s2)
{
  // Clamp targets to actuator bounds before ramping.
  double clamped_target_v = std::clamp(target_linear_velocity_m_s, min_linear_velocity_m_s_, max_linear_velocity_m_s_);
  double clamped_target_omega =
    std::clamp(target_angular_velocity_rad_s, min_angular_velocity_rad_s_, max_angular_velocity_rad_s_);

  double new_linear_velocity_m_s =
    rampedAdvance(current_linear_velocity_m_s, clamped_target_v, linear_acceleration_m_s2, dt_s);
  double new_angular_velocity_rad_s =
    rampedAdvance(current_angular_velocity_rad_s, clamped_target_omega, angular_acceleration_rad_s2, dt_s);

  DifferentialDriveWheelVelocities wheels =
    model_.bodyVelocityToWheelVelocities(new_linear_velocity_m_s, new_angular_velocity_rad_s);

  // Euler pose update (heading taken at start of step, matches bicycle/articulated projectors).
  Pose2D new_pose{
    current_pose.x + new_linear_velocity_m_s * std::cos(current_pose.theta) * dt_s,
    current_pose.y + new_linear_velocity_m_s * std::sin(current_pose.theta) * dt_s,
    normalizeAngle(current_pose.theta + new_angular_velocity_rad_s * dt_s)};

  return DifferentialDriveProjectedState{dt_s, new_pose, new_linear_velocity_m_s, new_angular_velocity_rad_s, wheels};
}

std::vector<DifferentialDriveProjectedState> DifferentialDriveProjector::project(
  double horizon_s,
  double dt_s,
  const Pose2D & initial_pose,
  double initial_linear_velocity_m_s,
  double initial_angular_velocity_rad_s,
  double target_linear_velocity_m_s,
  double target_angular_velocity_rad_s,
  double linear_acceleration_m_s2,
  double angular_acceleration_rad_s2)
{
  std::vector<DifferentialDriveProjectedState> trajectory;
  if (dt_s <= 0.0 || horizon_s < 0.0) {
    return trajectory;
  }

  DifferentialDriveWheelVelocities initial_wheels =
    model_.bodyVelocityToWheelVelocities(initial_linear_velocity_m_s, initial_angular_velocity_rad_s);
  trajectory.push_back(DifferentialDriveProjectedState{
    0.0, initial_pose, initial_linear_velocity_m_s, initial_angular_velocity_rad_s, initial_wheels});

  std::size_t n_steps = static_cast<std::size_t>(std::ceil(horizon_s / dt_s));
  trajectory.reserve(n_steps + 1);

  Pose2D pose = initial_pose;
  double linear_velocity_m_s = initial_linear_velocity_m_s;
  double angular_velocity_rad_s = initial_angular_velocity_rad_s;
  for (std::size_t i = 0; i < n_steps; ++i) {
    DifferentialDriveProjectedState s = step(
      dt_s,
      pose,
      linear_velocity_m_s,
      angular_velocity_rad_s,
      target_linear_velocity_m_s,
      target_angular_velocity_rad_s,
      linear_acceleration_m_s2,
      angular_acceleration_rad_s2);
    s.time_s = static_cast<double>(i + 1) * dt_s;
    pose = s.pose;
    linear_velocity_m_s = s.linear_velocity_m_s;
    angular_velocity_rad_s = s.angular_velocity_rad_s;
    trajectory.push_back(s);
  }
  return trajectory;
}

}  // namespace polymath::kinematics
