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

#ifndef POLYMATH_KINEMATICS__ARTICULATED_PROJECTOR_HPP__
#define POLYMATH_KINEMATICS__ARTICULATED_PROJECTOR_HPP__

#include <vector>

#include "polymath_kinematics/articulated_model.hpp"
#include "polymath_kinematics/pose2d.hpp"

// TODO(naming): when this projector is in wide use, consider renaming the underlying
// ArticulatedModel methods for clarity:
//   - bodyVelocityToVehicleState     -> inverseKinematics
//   - articulationToAxleVelocities   -> forwardKinematics
// And dropping the _m / _rad / _rad_s unit suffixes from struct fields (units in docs).

namespace polymath::kinematics
{

/// @brief One sample of an articulated-model projection. Pose is tracked at the rear segment.
struct ArticulatedProjectedState
{
  double time_s;  ///< Elapsed time from the start of the projection
  Pose2D pose;  ///< Rear-segment pose at this sample
  double articulation_angle_rad;  ///< Post-ramp articulation angle (gamma) in radians
  double linear_velocity_m_s;  ///< Commanded linear velocity in m/s
  double angular_velocity_rad_s;  ///< Rear-axle turning rate used for theta integration
  ArticulatedVehicleState vehicle_state;  ///< Full kinematic snapshot (wheel speeds + turning radii)
  Footprint front_footprint;  ///< World-frame front-body polygon; empty if no footprint dims set
  Footprint rear_footprint;  ///< World-frame rear-body polygon; empty if no footprint dims set
};

/// @brief Forward-projection wrapper around ArticulatedModel. Ramps articulation angle (gamma)
/// toward a target at a bounded rate (gamma-dot), clamping the target to [min, max] first.
/// Pose is integrated with Euler at the rear axle.
class ArticulatedProjector
{
public:
  /// @brief Construct a projector with articulation-angle limits and (optionally) footprint dims.
  ///
  /// Footprint dimensions describe the two vehicle bodies relative to the articulation joint and
  /// are owned by the projector (the kinematic model stays dimension-free). Each body is a
  /// rectangle: the front body extends `front_joint_to_bumper_m` ahead of the joint along the
  /// front-body heading; the rear body extends `rear_joint_to_bumper_m` behind the joint along
  /// the rear-body heading. A body width <= 0 means "unset/invalid": that body's footprint is
  /// emitted empty and projection proceeds normally (never throws).
  /// @param model Articulated kinematics model (stored by value)
  /// @param min_articulation_angle_rad Minimum allowed articulation angle (typically negative)
  /// @param max_articulation_angle_rad Maximum allowed articulation angle (typically positive)
  /// @param front_joint_to_bumper_m Front-body length from joint to front bumper (m)
  /// @param front_body_width_m Front-body width (m); <= 0 disables the front footprint
  /// @param rear_joint_to_bumper_m Rear-body length from joint to rear bumper (m)
  /// @param rear_body_width_m Rear-body width (m); <= 0 disables the rear footprint
  ArticulatedProjector(
    ArticulatedModel model,
    double min_articulation_angle_rad,
    double max_articulation_angle_rad,
    double front_joint_to_bumper_m = 0.0,
    double front_body_width_m = 0.0,
    double rear_joint_to_bumper_m = 0.0,
    double rear_body_width_m = 0.0)
  : model_(model)
  , min_articulation_angle_rad_(min_articulation_angle_rad)
  , max_articulation_angle_rad_(max_articulation_angle_rad)
  , front_joint_to_bumper_m_(front_joint_to_bumper_m)
  , front_body_width_m_(front_body_width_m)
  , rear_joint_to_bumper_m_(rear_joint_to_bumper_m)
  , rear_body_width_m_(rear_body_width_m)
  {}

  ~ArticulatedProjector() = default;

  /// @brief Advance the vehicle one time step.
  /// The articulation angle ramps from the current value toward clamp(target, min, max) at
  /// |articulation_rate_rad_s| per second, never overshooting. Pose is integrated with Euler
  /// using the post-ramp angle.
  /// @param dt_s Step duration in seconds (must be > 0)
  /// @param current_pose Rear-segment pose at the start of the step
  /// @param current_articulation_angle_rad Articulation angle at the start of the step
  /// @param target_articulation_angle_rad Desired articulation angle (clamped to [min, max] internally)
  /// @param articulation_rate_rad_s Magnitude of the ramp rate in rad/s (sign is ignored)
  /// @param linear_velocity_m_s Commanded linear velocity in m/s
  /// @return Projected state at the end of the step (time_s = dt_s)
  ArticulatedProjectedState step(
    double dt_s,
    const Pose2D & current_pose,
    double current_articulation_angle_rad,
    double target_articulation_angle_rad,
    double articulation_rate_rad_s,
    double linear_velocity_m_s);

  /// @brief Project a trajectory forward over `horizon_s` at `dt_s` steps.
  /// Element 0 is the initial state (time_s=0); element N is the final state.
  /// Trajectory length is ceil(horizon_s / dt_s) + 1.
  /// @return Sequence of timestamped states (initial state included as element 0)
  std::vector<ArticulatedProjectedState> project(
    double horizon_s,
    double dt_s,
    const Pose2D & initial_pose,
    double initial_articulation_angle_rad,
    double target_articulation_angle_rad,
    double articulation_rate_rad_s,
    double linear_velocity_m_s);

  const ArticulatedModel & get_model() const
  {
    return model_;
  }

  double get_min_articulation_angle_rad() const
  {
    return min_articulation_angle_rad_;
  }

  double get_max_articulation_angle_rad() const
  {
    return max_articulation_angle_rad_;
  }

  double get_front_joint_to_bumper_m() const
  {
    return front_joint_to_bumper_m_;
  }

  double get_front_body_width_m() const
  {
    return front_body_width_m_;
  }

  double get_rear_joint_to_bumper_m() const
  {
    return rear_joint_to_bumper_m_;
  }

  double get_rear_body_width_m() const
  {
    return rear_body_width_m_;
  }

private:
  /// @brief Fill state.front_footprint / state.rear_footprint (world frame) from the rear-segment
  /// pose and articulation angle already populated on `state`. A body with width <= 0 yields an
  /// empty footprint.
  /// TODO: (zeerek) surface footprint-invalid to the user (e.g. a validity flag / status enum on
  /// the projected state) instead of silently emitting an empty Footprint.
  void fillFootprints(ArticulatedProjectedState & state) const;

  ArticulatedModel model_;
  double min_articulation_angle_rad_;
  double max_articulation_angle_rad_;
  double front_joint_to_bumper_m_;
  double front_body_width_m_;
  double rear_joint_to_bumper_m_;
  double rear_body_width_m_;
};

}  // namespace polymath::kinematics

#endif  // POLYMATH_KINEMATICS__ARTICULATED_PROJECTOR_HPP__
