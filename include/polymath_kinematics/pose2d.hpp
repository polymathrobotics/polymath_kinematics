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

namespace polymath::kinematics
{

/// @brief Planar pose: position (x, y) and heading theta (radians, CCW from +x).
struct Pose2D
{
  double x;
  double y;
  double theta;
};

/// @brief Wrap an angle to [-pi, pi]
inline double normalizeAngle(double angle)
{
  constexpr double kTwoPi = 2.0 * M_PI;
  double a = std::fmod(angle + M_PI, kTwoPi);
  if (a < 0.0) {
    a += kTwoPi;
  }
  return a - M_PI;
}

}  // namespace polymath::kinematics

#endif  // POLYMATH_KINEMATICS__POSE2D_HPP__
