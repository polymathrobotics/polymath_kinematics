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

#ifndef POLYMATH_KINEMATICS__POSE2D_HPP__
#define POLYMATH_KINEMATICS__POSE2D_HPP__

#include <cmath>
#include <vector>

namespace polymath::kinematics
{

/// @brief Planar pose: position (x, y) and heading theta (radians, CCW from +x).
struct Pose2D
{
  double x;
  double y;
  double theta;
};

/// @brief A planar point (world frame, metres).
struct Point2D
{
  double x;
  double y;
};

/// @brief A vehicle footprint as a world-frame polygon: 4 corners, counter-clockwise,
/// NOT closed (the first corner is not repeated; consumers close it if needed). An empty
/// footprint means "not computed / invalid" (e.g. when no footprint dimensions were given).
using Footprint = std::vector<Point2D>;

/// @brief Wrap an angle to [-pi, pi].
/// std::remainder(angle, 2*pi) maps to the [-pi, pi] interval directly (note: exactly +pi
/// maps to +pi, where the old fmod-based implementation returned -pi; both denote the same angle).
inline double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

}  // namespace polymath::kinematics

#endif  // POLYMATH_KINEMATICS__POSE2D_HPP__
