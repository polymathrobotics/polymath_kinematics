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

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "polymath_kinematics/articulated_projector.hpp"
#include "polymath_kinematics_ros2/articulated_projector_params.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace polymath::kinematics::ros2
{

/// ROS 2 lifecycle wrapper around polymath_kinematics::ArticulatedProjector.
///
/// The node tracks the vehicle's measured articulation angle from a JointState topic and the
/// commanded body velocity from a cmd_vel topic. Every command produces a fresh forward projection
/// over `projection.horizon_s` at `projection.time_step_s` steps, starting from the identity pose
/// and the measured articulation angle, and ramping toward the articulation angle the command asks
/// for. The result is held on the node for getLastProjection() and published two ways: as a
/// MarkerArray on `projected_footprints` outlining both bodies at every sample, and as a Path on
/// `projected_path` tracing the reference axle through those same samples.
class ArticulatedProjectorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /// Construct the node.
  /// \param options Node options supplied by rclcpp or by a component container.
  explicit ArticulatedProjectorNode(const rclcpp::NodeOptions & options);

  /// Build the kinematic model and projector from parameters, and create the subscriptions.
  /// \param state The lifecycle state being transitioned from.
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /// Begin projecting on incoming commands.
  /// \param state The lifecycle state being transitioned from.
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /// Stop projecting on incoming commands.
  /// \param state The lifecycle state being transitioned from.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /// Tear down the subscriptions, the projector, and any cached projection.
  /// \param state The lifecycle state being transitioned from.
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /// \return A copy of the most recent projection, or an empty vector if none has been computed.
  std::vector<polymath::kinematics::ArticulatedProjectedState> getLastProjection() const;

  /// \return The most recently measured articulation angle in radians (0.0 before the first
  /// JointState message naming the configured joint arrives).
  double getArticulationAngleRad() const;

private:
  /// Latch the articulation angle from the joint named by the `articulation_joint_name` parameter.
  /// Messages that do not carry that joint (or carry no position for it) are ignored.
  /// \param msg The incoming joint state.
  void onJointState(const sensor_msgs::msg::JointState & msg);

  /// Project the trajectory the command implies from the measured articulation angle, and publish
  /// the footprint markers for it.
  /// \param msg The incoming velocity command.
  void onCmdVel(const geometry_msgs::msg::TwistStamped & msg);

  /// Outline the front and rear body footprints at every sample of `projection`. Samples whose
  /// footprint is unset contribute no marker.
  /// \param projection Projection to draw, in the frame named by `visualization.frame_id`.
  /// \return Markers led by a DELETEALL that clears the previous publication.
  std::unique_ptr<visualization_msgs::msg::MarkerArray> produceProjectedFootprintMarkers(
    const std::vector<polymath::kinematics::ArticulatedProjectedState> & projection) const;

  /// Trace the reference axle through `projection`, one pose per sample including the initial one.
  /// The axle is the one named by `projector.axle_reference`, and each pose's yaw is the heading of
  /// the body that axle belongs to.
  /// \param projection Projection to trace, in the frame named by `visualization.frame_id`.
  /// \return A Path holding every sample, undecimated.
  std::unique_ptr<nav_msgs::msg::Path> produceProjectedPath(
    const std::vector<polymath::kinematics::ArticulatedProjectedState> & projection) const;

  /// The underlying polymath_kinematics projector. Null until on_configure() succeeds.
  std::unique_ptr<polymath::kinematics::ArticulatedProjector> projector_;

  /// Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;

  /// Publishers
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr footprint_marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  /// Guards the state shared between the two subscription callbacks and the accessors, so the node
  /// stays correct under a multi-threaded executor.
  mutable std::mutex state_mutex_;

  /// Most recent measured articulation angle (gamma) in radians.
  double articulation_angle_rad_{0.0};

  /// Set once a JointState naming the configured joint has supplied an angle. Read outside
  /// state_mutex_ by the logging in both callbacks.
  std::atomic<bool> articulation_angle_seen_{false};

  /// Most recent projection, one entry per time step including the initial state.
  std::vector<polymath::kinematics::ArticulatedProjectedState> last_projection_;

  /// Parameters
  std::shared_ptr<articulated_projector::ParamListener> param_listener_;
  articulated_projector::Params params_;
};

}  // namespace polymath::kinematics::ros2
