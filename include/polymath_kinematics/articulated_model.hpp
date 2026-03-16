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

#ifndef POLYMATH_KINEMATICS__ARTICULATED_MODEL_HPP__
#define POLYMATH_KINEMATICS__ARTICULATED_MODEL_HPP__

namespace polymath::kinematics
{

/// @brief Complete vehicle state for an articulated vehicle
struct ArticulatedVehicleState
{
  double articulation_angle_rad;
  double linear_velocity_m_s;
  double front_right_wheel_speed_rad_s;
  double front_left_wheel_speed_rad_s;
  double rear_right_wheel_speed_rad_s;
  double rear_left_wheel_speed_rad_s;
  double front_axle_turning_radius_m;
  double rear_axle_turning_radius_m;
};

/// @brief Axle turning velocities for an articulated vehicle
struct ArticulatedAxleVelocities
{
  double linear_velocity_m_s;
  double front_axle_turning_velocity_rad_s;
  double rear_axle_turning_velocity_rad_s;
};

/// @brief Kinematic model for articulated vehicles (e.g., wheel loaders, articulated dump trucks)
/// @note TODO: (Zeerek) We want to add articulated angle turning velocity into our estimations
class ArticulatedModel
{
public:
  /// @brief Construct an articulated vehicle model
  /// @param articulation_to_front_axle_m Distance from articulation joint to front axle
  /// @param articulation_to_rear_axle_m Distance from articulation joint to rear axle
  /// @param front_track_width_m Width between front wheels
  /// @param rear_track_width_m Width between rear wheels
  /// @param front_wheel_radius_m Radius of front wheels
  /// @param rear_wheel_radius_m Radius of rear wheels
  ArticulatedModel(
    double articulation_to_front_axle_m,
    double articulation_to_rear_axle_m,
    double front_track_width_m,
    double rear_track_width_m,
    double front_wheel_radius_m,
    double rear_wheel_radius_m)
  : articulation_to_front_axle_m_(articulation_to_front_axle_m)
  , articulation_to_rear_axle_m_(articulation_to_rear_axle_m)
  , front_track_width_m_(front_track_width_m)
  , rear_track_width_m_(rear_track_width_m)
  , front_wheel_radius_m_(front_wheel_radius_m)
  , rear_wheel_radius_m_(rear_wheel_radius_m) {};

  ~ArticulatedModel() = default;

  /// @brief Convert body velocity to vehicle state (articulation angle and wheel speeds)
  /// @param linear_velocity_m_s Desired linear velocity in m/s
  /// @param angular_velocity_rad_s Desired angular velocity in rad/s
  /// @return Vehicle state including required articulation angle and wheel speeds
  ArticulatedVehicleState bodyVelocityToVehicleState(double linear_velocity_m_s, double angular_velocity_rad_s);

  /// @brief Convert articulation state to axle turning velocities
  /// @param linear_velocity_m_s Current linear velocity in m/s
  /// @param articulation_angle_rad Current articulation angle in radians
  /// @return Axle turning velocities for front and rear axles
  ArticulatedAxleVelocities articulationToAxleVelocities(double linear_velocity_m_s, double articulation_angle_rad);

  double get_articulation_to_front_axle_m() const
  {
    return articulation_to_front_axle_m_;
  }

  double get_articulation_to_rear_axle_m() const
  {
    return articulation_to_rear_axle_m_;
  }

  double get_front_track_width_m() const
  {
    return front_track_width_m_;
  }

  double get_rear_track_width_m() const
  {
    return rear_track_width_m_;
  }

  double get_front_wheel_radius_m() const
  {
    return front_wheel_radius_m_;
  }

  double get_rear_wheel_radius_m() const
  {
    return rear_wheel_radius_m_;
  }

private:
  double articulation_to_front_axle_m_;
  double articulation_to_rear_axle_m_;
  double front_track_width_m_;
  double rear_track_width_m_;
  double front_wheel_radius_m_;
  double rear_wheel_radius_m_;

  /// @brief Articulation turning velocity once calculated or available
  /// TODO: (Zeerek) Add ability to pass this in when generating estimations
  static constexpr double articulation_turning_velocity_rad_s_ = 0.0;

  /// @brief Threshold below which a velocity is treated as zero to avoid numerical
  /// singularities (e.g. 0/0 in acos, or inf*0 in wheel speed calculations)
  static constexpr double ZERO_VELOCITY_THRESHOLD = 1e-9;
};

}  // namespace polymath::kinematics

#endif  // POLYMATH_KINEMATICS__ARTICULATED_MODEL_HPP__
