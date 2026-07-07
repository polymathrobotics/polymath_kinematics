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

#include "polymath_kinematics/articulated_projector.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace polymath::kinematics
{

ArticulatedProjectedState ArticulatedProjector::step(
  double dt_s,
  const Pose2D & current_pose,
  double current_articulation_angle_rad,
  double target_articulation_angle_rad,
  double articulation_rate_rad_s,
  double linear_velocity_m_s)
{
  // Clamp target into the joint's mechanical bounds before ramping.
  double clamped_target =
    std::clamp(target_articulation_angle_rad, min_articulation_angle_rad_, max_articulation_angle_rad_);

  // Slew toward the clamped target at |rate| per second, never overshooting.
  double max_delta = std::abs(articulation_rate_rad_s) * dt_s;
  double delta = clamped_target - current_articulation_angle_rad;
  if (std::abs(delta) > max_delta) {
    delta = std::copysign(max_delta, delta);
  }
  double new_articulation_angle_rad = current_articulation_angle_rad + delta;
  // Actual gamma-dot realized during this step (zero once the angle pins at clamped_target).
  double actual_articulation_rate_rad_s = delta / dt_s;

  // Rear-axle turning velocity drives theta integration; feed the realized gamma-dot in.
  ArticulatedAxleVelocities axle = model_.articulationToAxleVelocities(
    linear_velocity_m_s, new_articulation_angle_rad, actual_articulation_rate_rad_s);
  double angular_velocity_rad_s = axle.rear_axle_turning_velocity_rad_s;

  // Full vehicle state (wheel speeds + turning radii) for the snapshot.
  ArticulatedVehicleState inner =
    model_.bodyVelocityToVehicleState(linear_velocity_m_s, angular_velocity_rad_s, actual_articulation_rate_rad_s);

  // The public pose (base_link) is the articulation joint, but motion is integrated at the rear
  // axle. Convert joint -> rear axle, Euler-step the rear axle, then convert back to the joint.
  const double rear_axle_to_joint = model_.get_articulation_to_rear_axle_m();
  const double rear_x = current_pose.x - rear_axle_to_joint * std::cos(current_pose.theta);
  const double rear_y = current_pose.y - rear_axle_to_joint * std::sin(current_pose.theta);

  const double new_rear_x = rear_x + linear_velocity_m_s * std::cos(current_pose.theta) * dt_s;
  const double new_rear_y = rear_y + linear_velocity_m_s * std::sin(current_pose.theta) * dt_s;
  const double new_theta = normalizeAngle(current_pose.theta + angular_velocity_rad_s * dt_s);

  Pose2D new_pose{
    new_rear_x + rear_axle_to_joint * std::cos(new_theta),
    new_rear_y + rear_axle_to_joint * std::sin(new_theta),
    new_theta};

  ArticulatedProjectedState state{
    dt_s, new_pose, new_articulation_angle_rad, linear_velocity_m_s, angular_velocity_rad_s, inner, {}, {}};
  fillFootprints(state);
  return state;
}

void ArticulatedProjector::fillFootprints(ArticulatedProjectedState & state) const
{
  // Pose (base_link) is the articulation joint; pose.theta is the rear-body heading. The front
  // body is rotated relative to the rear by the articulation angle (theta_front = theta_rear +
  // gamma; see derivations/articulated_model.md).
  const double rear_theta = state.pose.theta;
  const double front_theta = rear_theta + state.articulation_angle_rad;
  const double joint_x = state.pose.x;
  const double joint_y = state.pose.y;

  // Rectangle corners (CCW, open) given body-frame x extent [x_lo, x_hi] and width, placed at the
  // joint and rotated by `heading`. Returns empty when the width is unset/invalid (<= 0).
  auto make_body = [joint_x, joint_y](double heading, double x_lo, double x_hi, double width) {
    Footprint corners;
    if (width <= 0.0) {
      return corners;
    }
    const double half_w = width / 2.0;
    const double cos_h = std::cos(heading);
    const double sin_h = std::sin(heading);
    const double local_x[4] = {x_lo, x_hi, x_hi, x_lo};
    const double local_y[4] = {-half_w, -half_w, half_w, half_w};
    corners.reserve(4);
    for (int i = 0; i < 4; ++i) {
      corners.push_back(Point2D{
        joint_x + cos_h * local_x[i] - sin_h * local_y[i],
        joint_y + sin_h * local_x[i] + cos_h * local_y[i]});
    }
    return corners;
  };

  state.front_footprint = make_body(front_theta, 0.0, front_joint_to_bumper_m_, front_body_width_m_);
  state.rear_footprint = make_body(rear_theta, -rear_joint_to_bumper_m_, 0.0, rear_body_width_m_);
}

std::vector<ArticulatedProjectedState> ArticulatedProjector::project(
  double horizon_s,
  double dt_s,
  const Pose2D & initial_pose,
  double initial_articulation_angle_rad,
  double target_articulation_angle_rad,
  double articulation_rate_rad_s,
  double linear_velocity_m_s)
{
  std::vector<ArticulatedProjectedState> trajectory;
  if (dt_s <= 0.0 || horizon_s < 0.0) {
    return trajectory;
  }

  // Seed element 0 with the initial state.
  ArticulatedAxleVelocities initial_axle =
    model_.articulationToAxleVelocities(linear_velocity_m_s, initial_articulation_angle_rad);
  double initial_omega = initial_axle.rear_axle_turning_velocity_rad_s;
  ArticulatedVehicleState initial_inner = model_.bodyVelocityToVehicleState(linear_velocity_m_s, initial_omega);
  ArticulatedProjectedState initial_state{
    0.0, initial_pose, initial_articulation_angle_rad, linear_velocity_m_s, initial_omega, initial_inner, {}, {}};
  fillFootprints(initial_state);
  trajectory.push_back(initial_state);

  std::size_t n_steps = static_cast<std::size_t>(std::ceil(horizon_s / dt_s));
  trajectory.reserve(n_steps + 1);

  Pose2D pose = initial_pose;
  double articulation_angle = initial_articulation_angle_rad;
  for (std::size_t i = 0; i < n_steps; ++i) {
    ArticulatedProjectedState s =
      step(dt_s, pose, articulation_angle, target_articulation_angle_rad, articulation_rate_rad_s, linear_velocity_m_s);
    s.time_s = static_cast<double>(i + 1) * dt_s;
    pose = s.pose;
    articulation_angle = s.articulation_angle_rad;
    trajectory.push_back(s);
  }
  return trajectory;
}

}  // namespace polymath::kinematics
