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

#include <utility>
#include <vector>

#include "polymath_kinematics/articulated_model.hpp"
#include "polymath_kinematics/pose2d.hpp"

namespace polymath::kinematics
{

/// @brief One sample of an articulated-model projection. `pose` sits at the projector's reference
/// axle, with theta the heading of the body that axle belongs to. Motion is integrated at the rear
/// axle internally.
struct ArticulatedProjectedState
{
  double time_s;  ///< Elapsed time from the start of the projection
  Pose2D pose;  ///< Reference-axle pose; theta = heading of that axle's body
  double articulation_angle_rad;  ///< Post-ramp articulation angle (gamma) in radians
  double linear_velocity_m_s;  ///< Commanded linear velocity in m/s
  double angular_velocity_rad_s;  ///< Rear-axle turning rate used for theta integration
  ArticulatedVehicleState vehicle_state;  ///< Full kinematic snapshot (wheel speeds + turning radii)
  Pose2D joint_pose;  ///< Articulation-joint (base_link) pose; theta = rear-body heading
  Footprint front_footprint;  ///< World-frame front-body polygon; empty if none was set
  Footprint rear_footprint;  ///< World-frame rear-body polygon; empty if none was set
};

/// @brief Forward-projection wrapper around ArticulatedModel. Ramps articulation angle (gamma)
/// toward a target at a bounded rate (gamma-dot), clamping the target to [min, max] first.
///
/// Poses are measured at the axle named by `axle_reference`, with theta the heading of the body
/// that axle belongs to (rear-body heading for REAR, front-body heading = theta_rear + gamma for
/// FRONT). Motion is always integrated at the rear axle; the reference conversion is applied on
/// input and output. The articulation-joint pose is reported alongside on every sample.
class ArticulatedProjector
{
public:
  /// @brief Construct a projector with articulation-angle limits and (optionally) body footprints.
  ///
  /// Each body carries its own arbitrary polygon, expressed in the frame of **its own** axle: the
  /// front polygon about the front axle with +x along the front-body heading, the rear polygon
  /// about the rear axle with +x along the rear-body heading. This is the only anchoring that stays
  /// rigid as the joint articulates. Footprints are owned by the projector, so the kinematic model
  /// stays dimension-free. An empty polygon means "unset": that body's footprint is emitted empty
  /// and projection proceeds normally (never throws). Use `rectangleFootprint()` for boxy bodies.
  /// @param model Articulated kinematics model (stored by value)
  /// @param min_articulation_angle_rad Minimum allowed articulation angle (typically negative)
  /// @param max_articulation_angle_rad Maximum allowed articulation angle (typically positive)
  /// @param axle_reference Axle that reported poses are measured from
  /// @param front_footprint Front-body polygon in the front-axle frame
  /// @param rear_footprint Rear-body polygon in the rear-axle frame
  ArticulatedProjector(
    ArticulatedModel model,
    double min_articulation_angle_rad,
    double max_articulation_angle_rad,
    AxleReference axle_reference = AxleReference::REAR,
    Footprint front_footprint = {},
    Footprint rear_footprint = {})
  : model_(model)
  , min_articulation_angle_rad_(min_articulation_angle_rad)
  , max_articulation_angle_rad_(max_articulation_angle_rad)
  , axle_reference_(axle_reference)
  , front_footprint_(std::move(front_footprint))
  , rear_footprint_(std::move(rear_footprint))
  {}

  ~ArticulatedProjector() = default;

  /// @brief Advance the vehicle one time step.
  /// The articulation angle ramps from the current value toward clamp(target, min, max) at
  /// |articulation_rate_rad_s| per second, never overshooting. Pose is integrated with Euler
  /// using the post-ramp angle.
  /// @param dt_s Step duration in seconds (must be > 0)
  /// @param current_pose Reference-axle pose at the start of the step
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

  AxleReference get_axle_reference() const
  {
    return axle_reference_;
  }

  const Footprint & get_front_footprint() const
  {
    return front_footprint_;
  }

  const Footprint & get_rear_footprint() const
  {
    return rear_footprint_;
  }

private:
  /// @brief Convert a pose at the reference axle to the rear axle, where motion is integrated.
  Pose2D toRearAxle(const Pose2D & reference_pose, double articulation_angle_rad) const;

  /// @brief Rear-axle pose -> articulation-joint pose (theta unchanged, still the rear heading).
  Pose2D jointFromRearAxle(const Pose2D & rear_axle_pose) const;

  /// @brief Rear-axle pose -> front-axle pose, whose theta is the front-body heading.
  Pose2D frontAxleFromRearAxle(const Pose2D & rear_axle_pose, double articulation_angle_rad) const;

  /// @brief Populate pose / joint_pose / both footprints from a rear-axle pose and gamma.
  void fillPosesAndFootprints(
    ArticulatedProjectedState & state, const Pose2D & rear_axle_pose, double articulation_angle_rad) const;

  ArticulatedModel model_;
  double min_articulation_angle_rad_;
  double max_articulation_angle_rad_;
  AxleReference axle_reference_;
  Footprint front_footprint_;
  Footprint rear_footprint_;
};

}  // namespace polymath::kinematics

#endif  // POLYMATH_KINEMATICS__ARTICULATED_PROJECTOR_HPP__
