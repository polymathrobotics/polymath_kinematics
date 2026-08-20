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

#include "polymath_kinematics_ros2/articulated_projector_node.hpp"

#include <algorithm>
#include <cstddef>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "magic_enum/magic_enum.hpp"
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(polymath::kinematics::ros2::ArticulatedProjectorNode)

namespace polymath::kinematics::ros2
{

namespace
{

/// Depth of the two command/feedback subscriptions. Both carry the latest sample only, so a short
/// queue is enough and keeps a backlog from projecting stale commands.
constexpr int SUBSCRIPTION_QUEUE_DEPTH = 1;

/// Reinterpret a flat [x0, y0, x1, y1, ...] parameter as a footprint polygon.
/// \param flat_xy Flat list of alternating x and y coordinates; an odd length is rejected.
/// \return The polygon, or an empty optional if `flat_xy` does not hold whole x,y pairs.
std::optional<Footprint> footprintFromFlatArray(const std::vector<double> & flat_xy)
{
  if (0 != flat_xy.size() % 2) {
    return std::nullopt;
  }
  Footprint footprint;
  footprint.reserve(flat_xy.size() / 2);
  for (size_t index = 0; index < flat_xy.size(); index += 2) {
    footprint.push_back(Point2D{flat_xy[index], flat_xy[index + 1]});
  }
  return footprint;
}

}  // namespace

ArticulatedProjectorNode::ArticulatedProjectorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("articulated_projector", options)
{
  param_listener_ = std::make_shared<articulated_projector::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();
}

ArticulatedProjectorNode::CallbackReturn ArticulatedProjectorNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;
  params_ = param_listener_->get_params();

  const std::optional<Footprint> front_footprint = footprintFromFlatArray(params_.projector.front_footprint);
  const std::optional<Footprint> rear_footprint = footprintFromFlatArray(params_.projector.rear_footprint);
  if (!front_footprint.has_value() || !rear_footprint.has_value()) {
    RCLCPP_ERROR(get_logger(), "footprint parameters must hold an even number of entries (flat x,y pairs)");
    return CallbackReturn::FAILURE;
  }

  const std::optional<AxleReference> axle_reference =
    magic_enum::enum_cast<AxleReference>(params_.projector.axle_reference, magic_enum::case_insensitive);
  if (!axle_reference.has_value()) {
    RCLCPP_ERROR(get_logger(), "axle_reference '%s' is not a known axle", params_.projector.axle_reference.c_str());
    return CallbackReturn::FAILURE;
  }

  if (params_.projection.time_step_s > params_.projection.horizon_s) {
    RCLCPP_ERROR(
      get_logger(),
      "projection.time_step_s (%f) exceeds projection.horizon_s (%f)",
      params_.projection.time_step_s,
      params_.projection.horizon_s);
    return CallbackReturn::FAILURE;
  }

  const ArticulatedModel model = ArticulatedModel(
    params_.model.articulation_to_front_axle_m,
    params_.model.articulation_to_rear_axle_m,
    params_.model.front_track_width_m,
    params_.model.rear_track_width_m,
    params_.model.front_wheel_radius_m,
    params_.model.rear_wheel_radius_m);

  projector_ = std::make_unique<ArticulatedProjector>(
    model,
    params_.projector.minimum_articulation_angle_rad,
    params_.projector.maximum_articulation_angle_rad,
    axle_reference.value(),
    front_footprint.value(),
    rear_footprint.value());

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", SUBSCRIPTION_QUEUE_DEPTH, [this](const sensor_msgs::msg::JointState & msg) { onJointState(msg); });
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "cmd_vel", SUBSCRIPTION_QUEUE_DEPTH, [this](const geometry_msgs::msg::TwistStamped & msg) { onCmdVel(msg); });

  RCLCPP_INFO(
    get_logger(),
    "configured: tracking joint '%s', projecting %.2fs ahead in %.3fs steps",
    params_.articulation_joint_name.c_str(),
    params_.projection.horizon_s,
    params_.projection.time_step_s);
  return CallbackReturn::SUCCESS;
}

ArticulatedProjectorNode::CallbackReturn ArticulatedProjectorNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "activated");
  return CallbackReturn::SUCCESS;
}

ArticulatedProjectorNode::CallbackReturn ArticulatedProjectorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "deactivated");
  return CallbackReturn::SUCCESS;
}

ArticulatedProjectorNode::CallbackReturn ArticulatedProjectorNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  joint_state_sub_.reset();
  cmd_vel_sub_.reset();
  projector_.reset();
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    articulation_angle_rad_ = 0.0;
    last_projection_.clear();
  }
  RCLCPP_INFO(get_logger(), "cleaned up");
  return CallbackReturn::SUCCESS;
}

std::vector<ArticulatedProjectedState> ArticulatedProjectorNode::getLastProjection() const
{
  const std::lock_guard<std::mutex> lock(state_mutex_);
  return last_projection_;
}

double ArticulatedProjectorNode::getArticulationAngleRad() const
{
  const std::lock_guard<std::mutex> lock(state_mutex_);
  return articulation_angle_rad_;
}

void ArticulatedProjectorNode::onJointState(const sensor_msgs::msg::JointState & msg)
{
  const auto joint = std::find(msg.name.begin(), msg.name.end(), params_.articulation_joint_name);
  if (msg.name.end() == joint) {
    return;
  }

  // position[] is allowed to be shorter than name[]: a joint can be reported with velocity/effort only.
  const size_t index = static_cast<size_t>(std::distance(msg.name.begin(), joint));
  if (index >= msg.position.size()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "joint '%s' carries no position", params_.articulation_joint_name.c_str());
    return;
  }

  const std::lock_guard<std::mutex> lock(state_mutex_);
  articulation_angle_rad_ = msg.position[index];
}

void ArticulatedProjectorNode::onCmdVel(const geometry_msgs::msg::TwistStamped & msg)
{
  if (lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE != get_current_state().id()) {
    return;
  }

  // The command is a body velocity; the projector steers by articulation angle, so ask the model
  // which articulation angle sustains that (v, omega) pair and ramp toward it.
  ArticulatedModel model = projector_->get_model();
  const ArticulatedVehicleState commanded = model.bodyVelocityToVehicleState(msg.twist.linear.x, msg.twist.angular.z);

  const std::lock_guard<std::mutex> lock(state_mutex_);
  last_projection_ = projector_->project(
    params_.projection.horizon_s,
    params_.projection.time_step_s,
    Pose2D{0.0, 0.0, 0.0},
    articulation_angle_rad_,
    commanded.articulation_angle_rad,
    params_.projector.articulation_rate_rad_s,
    msg.twist.linear.x);
}

}  // namespace polymath::kinematics::ros2
