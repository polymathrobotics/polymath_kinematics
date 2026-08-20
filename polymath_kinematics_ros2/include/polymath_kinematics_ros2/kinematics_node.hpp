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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace polymath::kinematics_ros2
{

/// Placeholder ROS 2 lifecycle wrapper around the polymath_kinematics models.
///
/// The node declares no parameters, topics, or services yet. It exists so the ROS 2 layer has a
/// build target, a component registration, and a lifecycle contract to grow into; every transition
/// callback is currently a no-op that reports SUCCESS.
class KinematicsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// Construct the node.
  /// \param options Node options supplied by rclcpp or by a component container.
  explicit KinematicsNode(const rclcpp::NodeOptions & options);

  /// Allocate resources. No-op until the node gains interfaces.
  /// \param state The lifecycle state being transitioned from.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /// Begin publishing. No-op until the node gains interfaces.
  /// \param state The lifecycle state being transitioned from.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /// Stop publishing. No-op until the node gains interfaces.
  /// \param state The lifecycle state being transitioned from.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /// Release resources. No-op until the node gains interfaces.
  /// \param state The lifecycle state being transitioned from.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
};

}  // namespace polymath::kinematics_ros2
