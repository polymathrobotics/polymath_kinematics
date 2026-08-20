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

#include "polymath_kinematics_ros2/kinematics_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace polymath::kinematics_ros2
{

KinematicsNode::KinematicsNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("kinematics_node", options)
{}

KinematicsNode::CallbackReturn KinematicsNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "configured (placeholder: nothing to set up yet)");
  return CallbackReturn::SUCCESS;
}

KinematicsNode::CallbackReturn KinematicsNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "activated (placeholder: nothing to publish yet)");
  return CallbackReturn::SUCCESS;
}

KinematicsNode::CallbackReturn KinematicsNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "deactivated");
  return CallbackReturn::SUCCESS;
}

KinematicsNode::CallbackReturn KinematicsNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "cleaned up");
  return CallbackReturn::SUCCESS;
}

}  // namespace polymath::kinematics_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(polymath::kinematics_ros2::KinematicsNode)
