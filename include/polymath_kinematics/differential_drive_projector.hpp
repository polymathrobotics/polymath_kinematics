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

#ifndef POLYMATH_KINEMATICS__DIFFERENTIAL_DRIVE_PROJECTOR_HPP__
#define POLYMATH_KINEMATICS__DIFFERENTIAL_DRIVE_PROJECTOR_HPP__

#include <vector>

#include "polymath_kinematics/differential_drive_model.hpp"
#include "polymath_kinematics/pose2d.hpp"

namespace polymath::kinematics
{

/// @brief One sample of a differential-drive-model projection.
struct DifferentialDriveProjectedState
{
  double time_s;  ///< Elapsed time from the start of the projection
  Pose2D pose;  ///< Body pose at this sample
  double linear_velocity_m_s;  ///< Post-ramp body linear velocity in m/s
  double angular_velocity_rad_s;  ///< Post-ramp body angular velocity in rad/s
  DifferentialDriveWheelVelocities wheel_velocities;  ///< Wheel speeds derived from the body command
};

/// @brief Forward-projection wrapper around DifferentialDriveModel that ramps the body command
/// (linear_velocity, angular_velocity) toward targets at bounded accelerations, clamping each
/// to its [min, max] range before ramping. Pose is integrated with Euler.
class DifferentialDriveProjector
{
public:
  /// @brief Construct a projector with body-velocity limits.
  /// @param model Differential drive kinematics model (stored by value)
  /// @param min_linear_velocity_m_s Minimum allowed linear velocity (typically negative for reverse)
  /// @param max_linear_velocity_m_s Maximum allowed linear velocity
  /// @param min_angular_velocity_rad_s Minimum allowed angular velocity (typically negative)
  /// @param max_angular_velocity_rad_s Maximum allowed angular velocity
  DifferentialDriveProjector(
    DifferentialDriveModel model,
    double min_linear_velocity_m_s,
    double max_linear_velocity_m_s,
    double min_angular_velocity_rad_s,
    double max_angular_velocity_rad_s)
  : model_(model)
  , min_linear_velocity_m_s_(min_linear_velocity_m_s)
  , max_linear_velocity_m_s_(max_linear_velocity_m_s)
  , min_angular_velocity_rad_s_(min_angular_velocity_rad_s)
  , max_angular_velocity_rad_s_(max_angular_velocity_rad_s)
  {}

  ~DifferentialDriveProjector() = default;

  /// @brief Advance the vehicle one time step.
  /// Linear and angular velocities ramp from their current values toward
  /// clamp(target, min, max) at the corresponding |acceleration| per second,
  /// never overshooting. Pose is integrated with Euler using the post-ramp body command.
  /// @param dt_s Step duration in seconds (must be > 0)
  /// @param current_pose Pose at the start of the step
  /// @param current_linear_velocity_m_s Linear velocity at the start of the step
  /// @param current_angular_velocity_rad_s Angular velocity at the start of the step
  /// @param target_linear_velocity_m_s Desired linear velocity (clamped internally)
  /// @param target_angular_velocity_rad_s Desired angular velocity (clamped internally)
  /// @param linear_acceleration_m_s2 Magnitude of the linear ramp rate (sign is ignored)
  /// @param angular_acceleration_rad_s2 Magnitude of the angular ramp rate (sign is ignored)
  /// @return Projected state at the end of the step (time_s = dt_s)
  DifferentialDriveProjectedState step(
    double dt_s,
    const Pose2D & current_pose,
    double current_linear_velocity_m_s,
    double current_angular_velocity_rad_s,
    double target_linear_velocity_m_s,
    double target_angular_velocity_rad_s,
    double linear_acceleration_m_s2,
    double angular_acceleration_rad_s2);

  /// @brief Project a trajectory forward over `horizon_s` at `dt_s` steps.
  /// Element 0 is the initial state (time_s=0); element N is the final state.
  /// Trajectory length is ceil(horizon_s / dt_s) + 1.
  std::vector<DifferentialDriveProjectedState> project(
    double horizon_s,
    double dt_s,
    const Pose2D & initial_pose,
    double initial_linear_velocity_m_s,
    double initial_angular_velocity_rad_s,
    double target_linear_velocity_m_s,
    double target_angular_velocity_rad_s,
    double linear_acceleration_m_s2,
    double angular_acceleration_rad_s2);

  const DifferentialDriveModel & get_model() const
  {
    return model_;
  }

  double get_min_linear_velocity_m_s() const
  {
    return min_linear_velocity_m_s_;
  }

  double get_max_linear_velocity_m_s() const
  {
    return max_linear_velocity_m_s_;
  }

  double get_min_angular_velocity_rad_s() const
  {
    return min_angular_velocity_rad_s_;
  }

  double get_max_angular_velocity_rad_s() const
  {
    return max_angular_velocity_rad_s_;
  }

private:
  DifferentialDriveModel model_;
  double min_linear_velocity_m_s_;
  double max_linear_velocity_m_s_;
  double min_angular_velocity_rad_s_;
  double max_angular_velocity_rad_s_;
};

}  // namespace polymath::kinematics

#endif  // POLYMATH_KINEMATICS__DIFFERENTIAL_DRIVE_PROJECTOR_HPP__
