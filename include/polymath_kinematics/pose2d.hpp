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

/// @brief A vehicle footprint as a polygon of arbitrary vertex count, counter-clockwise and NOT
/// closed (the first vertex is not repeated; consumers close it if needed). Used both for the
/// body-frame outline handed to a projector and for the world-frame result it emits. An empty
/// footprint means "not set / not computed".
using Footprint = std::vector<Point2D>;

/// @brief Which axle a projector's poses and body-frame footprint are measured from.
enum class AxleReference
{
  FRONT,
  REAR
};

/// @brief Build a rectangular body-frame footprint, the common case for a boxy vehicle.
/// @param front_m Distance from the reference axle forward to the front edge (may be negative)
/// @param rear_m Distance from the reference axle back to the rear edge (positive = behind)
/// @param width_m Total body width; <= 0 returns an empty footprint
/// @return Corners CCW and open: rear-right, front-right, front-left, rear-left
inline Footprint rectangleFootprint(double front_m, double rear_m, double width_m)
{
  if (width_m <= 0.0) {
    return Footprint{};
  }
  const double half_w = width_m / 2.0;
  return Footprint{
    Point2D{-rear_m, -half_w}, Point2D{front_m, -half_w}, Point2D{front_m, half_w}, Point2D{-rear_m, half_w}};
}

/// @brief Transform a body-frame footprint into the world frame by `pose`.
inline Footprint transformFootprint(const Footprint & body_frame, const Pose2D & pose)
{
  Footprint world;
  world.reserve(body_frame.size());
  const double cos_t = std::cos(pose.theta);
  const double sin_t = std::sin(pose.theta);
  for (const Point2D & vertex : body_frame) {
    world.push_back(
      Point2D{pose.x + cos_t * vertex.x - sin_t * vertex.y, pose.y + sin_t * vertex.x + cos_t * vertex.y});
  }
  return world;
}

/// @brief Offset a pose along its own heading; negative `distance_m` moves backward.
inline Pose2D offsetAlongHeading(const Pose2D & pose, double distance_m)
{
  return Pose2D{pose.x + distance_m * std::cos(pose.theta), pose.y + distance_m * std::sin(pose.theta), pose.theta};
}

/// @brief Wrap an angle to [-pi, pi].
/// std::remainder(angle, 2*pi) maps to the [-pi, pi] interval directly (note: exactly +pi
/// maps to +pi, where the old fmod-based implementation returned -pi; both denote the same angle).
inline double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

}  // namespace polymath::kinematics

#endif  // POLYMATH_KINEMATICS__POSE2D_HPP__
