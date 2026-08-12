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

#ifndef POLYMATH_KINEMATICS__BICYCLE_PROJECTOR_HPP__
#define POLYMATH_KINEMATICS__BICYCLE_PROJECTOR_HPP__

#include <utility>
#include <vector>

#include "polymath_kinematics/bicycle_model.hpp"
#include "polymath_kinematics/pose2d.hpp"

namespace polymath::kinematics
{

/// @brief One sample of a bicycle-model projection: pose + steering + wheel speeds at a point in time.
struct BicycleProjectedState
{
  double time_s;  ///< Elapsed time from the start of the projection
  Pose2D pose;  ///< Body pose at the projector's reference axle; theta is the chassis heading
  double steering_angle_rad;  ///< Post-ramp steering angle in radians
  double linear_velocity_m_s;  ///< Commanded linear velocity in m/s
  double angular_velocity_rad_s;  ///< Body angular velocity used for theta integration
  BicycleSteeringState steering_state;  ///< Full kinematic snapshot (wheel speeds + turning radius)
  Footprint footprint;  ///< World-frame body polygon; empty if no footprint was set
};

/// @brief Forward-projection wrapper around BicycleModel that ramps steering toward a target at
/// a bounded rate, clamping the target to [min, max] before ramping, and integrates pose with Euler.
///
/// Poses and the body-frame footprint are both measured from the axle named by `axle_reference`.
/// Motion is always integrated at the rear axle (where the bicycle model is defined); a FRONT
/// reference is a rigid wheelbase offset along the chassis heading, applied on input and output.
class BicycleProjector
{
public:
  /// @brief Construct a projector with steering-angle limits and (optionally) a body footprint.
  ///
  /// The footprint is an arbitrary polygon in the body frame of `axle_reference`: +x along the
  /// chassis heading, +y to the left, origin at that axle. It is owned by the projector, so the
  /// kinematic model stays dimension-free. An empty footprint means "unset": projected states
  /// carry an empty footprint and projection proceeds normally (never throws). Use
  /// `rectangleFootprint()` for a boxy vehicle.
  /// @param model Bicycle kinematics model (stored by value)
  /// @param min_steering_angle_rad Minimum allowed steering angle (typically negative)
  /// @param max_steering_angle_rad Maximum allowed steering angle (typically positive)
  /// @param axle_reference Axle that poses and the footprint are measured from
  /// @param footprint Body-frame body polygon; empty disables footprint output
  BicycleProjector(
    BicycleModel model,
    double min_steering_angle_rad,
    double max_steering_angle_rad,
    AxleReference axle_reference = AxleReference::REAR,
    Footprint footprint = {})
  : model_(model)
  , min_steering_angle_rad_(min_steering_angle_rad)
  , max_steering_angle_rad_(max_steering_angle_rad)
  , axle_reference_(axle_reference)
  , footprint_(std::move(footprint))
  {}

  ~BicycleProjector() = default;

  /// @brief Advance the vehicle one time step.
  /// Steering ramps from current_steering_angle_rad toward clamp(target, min, max) at
  /// |steering_rate_rad_s| per second, never overshooting. Pose is integrated with Euler
  /// using the post-ramp angle.
  /// @param dt_s Step duration in seconds (must be > 0)
  /// @param current_pose Pose at the start of the step
  /// @param current_steering_angle_rad Steering angle at the start of the step
  /// @param target_steering_angle_rad Desired steering angle (clamped to [min, max] internally)
  /// @param steering_rate_rad_s Magnitude of the ramp rate in rad/s (sign is ignored)
  /// @param linear_velocity_m_s Commanded linear velocity in m/s
  /// @return Projected state at the end of the step (time_s = dt_s)
  BicycleProjectedState step(
    double dt_s,
    const Pose2D & current_pose,
    double current_steering_angle_rad,
    double target_steering_angle_rad,
    double steering_rate_rad_s,
    double linear_velocity_m_s);

  /// @brief Project a trajectory forward over `horizon_s` at `dt_s` steps.
  /// Element 0 is the initial state (time_s=0); element N is the final state.
  /// Trajectory length is ceil(horizon_s / dt_s) + 1.
  /// @return Sequence of timestamped states (initial state included as element 0)
  std::vector<BicycleProjectedState> project(
    double horizon_s,
    double dt_s,
    const Pose2D & initial_pose,
    double initial_steering_angle_rad,
    double target_steering_angle_rad,
    double steering_rate_rad_s,
    double linear_velocity_m_s);

  const BicycleModel & get_model() const
  {
    return model_;
  }

  double get_min_steering_angle_rad() const
  {
    return min_steering_angle_rad_;
  }

  double get_max_steering_angle_rad() const
  {
    return max_steering_angle_rad_;
  }

  AxleReference get_axle_reference() const
  {
    return axle_reference_;
  }

  const Footprint & get_footprint() const
  {
    return footprint_;
  }

private:
  /// @brief Convert a pose at the reference axle to the rear axle, where motion is integrated.
  Pose2D toRearAxle(const Pose2D & reference_pose) const;

  /// @brief Convert a rear-axle pose back out to the reference axle.
  Pose2D fromRearAxle(const Pose2D & rear_axle_pose) const;

  BicycleModel model_;
  double min_steering_angle_rad_;
  double max_steering_angle_rad_;
  AxleReference axle_reference_;
  Footprint footprint_;
};

}  // namespace polymath::kinematics

#endif  // POLYMATH_KINEMATICS__BICYCLE_PROJECTOR_HPP__
