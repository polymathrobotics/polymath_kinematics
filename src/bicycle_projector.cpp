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

#include "polymath_kinematics/bicycle_projector.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace polymath::kinematics
{

BicycleProjectedState BicycleProjector::step(
  double dt_s,
  const Pose2D & current_pose,
  double current_steering_angle_rad,
  double target_steering_angle_rad,
  double steering_rate_rad_s,
  double linear_velocity_m_s)
{
  // Clamp target into the actuator's bounds before ramping, so out-of-range commands
  // saturate at the limit rather than oscillating or overshooting.
  double clamped_target = std::clamp(target_steering_angle_rad, min_steering_angle_rad_, max_steering_angle_rad_);

  // Slew toward the clamped target at |rate| per second, never overshooting.
  double max_delta = std::abs(steering_rate_rad_s) * dt_s;
  double delta = clamped_target - current_steering_angle_rad;
  if (std::abs(delta) > max_delta) {
    delta = std::copysign(max_delta, delta);
  }
  double new_steering_angle_rad = current_steering_angle_rad + delta;

  // Forward kinematics with the post-ramp angle gives the body omega used for theta integration.
  BicycleBodyVelocity body_vel = model_.steeringToBodyVelocity(linear_velocity_m_s, new_steering_angle_rad);

  // Inverse kinematics populates wheel speeds + turning radius for the snapshot.
  BicycleSteeringState inner = model_.bodyVelocityToSteering(linear_velocity_m_s, body_vel.angular_velocity_rad_s);

  // Euler pose update (heading taken at start of step, matching the plan's integration choice).
  Pose2D new_pose{
    current_pose.x + linear_velocity_m_s * std::cos(current_pose.theta) * dt_s,
    current_pose.y + linear_velocity_m_s * std::sin(current_pose.theta) * dt_s,
    normalizeAngle(current_pose.theta + body_vel.angular_velocity_rad_s * dt_s)};

  return BicycleProjectedState{
    dt_s, new_pose, new_steering_angle_rad, linear_velocity_m_s, body_vel.angular_velocity_rad_s, inner};
}

std::vector<BicycleProjectedState> BicycleProjector::project(
  double horizon_s,
  double dt_s,
  const Pose2D & initial_pose,
  double initial_steering_angle_rad,
  double target_steering_angle_rad,
  double steering_rate_rad_s,
  double linear_velocity_m_s)
{
  std::vector<BicycleProjectedState> trajectory;
  if (dt_s <= 0.0 || horizon_s < 0.0) {
    return trajectory;
  }

  // Seed element 0 with the initial state so plots have a clean t=0 anchor.
  // Populate the steering_state field by running inverse kinematics on the implied omega.
  BicycleBodyVelocity initial_body = model_.steeringToBodyVelocity(linear_velocity_m_s, initial_steering_angle_rad);
  BicycleSteeringState initial_inner =
    model_.bodyVelocityToSteering(linear_velocity_m_s, initial_body.angular_velocity_rad_s);
  trajectory.push_back(BicycleProjectedState{
    0.0,
    initial_pose,
    initial_steering_angle_rad,
    linear_velocity_m_s,
    initial_body.angular_velocity_rad_s,
    initial_inner});

  std::size_t n_steps = static_cast<std::size_t>(std::ceil(horizon_s / dt_s));
  trajectory.reserve(n_steps + 1);

  Pose2D pose = initial_pose;
  double steering_angle = initial_steering_angle_rad;
  for (std::size_t i = 0; i < n_steps; ++i) {
    BicycleProjectedState s =
      step(dt_s, pose, steering_angle, target_steering_angle_rad, steering_rate_rad_s, linear_velocity_m_s);
    s.time_s = static_cast<double>(i + 1) * dt_s;
    pose = s.pose;
    steering_angle = s.steering_angle_rad;
    trajectory.push_back(s);
  }
  return trajectory;
}

}  // namespace polymath::kinematics
