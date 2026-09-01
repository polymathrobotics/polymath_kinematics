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
#include <cmath>
#include <cstddef>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "magic_enum.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/color_rgba.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(polymath::kinematics::ros2::ArticulatedProjectorNode)

namespace polymath::kinematics::ros2
{

namespace
{

/// Depth of the two command/feedback subscriptions. Both carry the latest sample only, so a short
/// queue is enough and keeps a backlog from projecting stale commands.
constexpr int SUBSCRIPTION_QUEUE_DEPTH = 1;

/// Depth of the marker and path publishers; each publication supersedes the last.
constexpr int MARKER_QUEUE_DEPTH = 1;

/// Height above the ground plane the outlines are drawn at.
constexpr double MARKER_Z_OFFSET_M = 0.001;

/// Marker namespace holding the front-body outlines.
constexpr const char * FRONT_MARKER_NAMESPACE = "projected_front_footprint";

/// Marker namespace holding the rear-body outlines.
constexpr const char * REAR_MARKER_NAMESPACE = "projected_rear_footprint";

/// Throttle period for the per-message diagnostic logs.
constexpr int LOG_THROTTLE_MS = 2000;

constexpr double RAD_TO_DEG = 57.29577951308232;

/// Render a JointState's name list for a log line.
std::string joinNames(const std::vector<std::string> & names)
{
  std::string joined;
  for (const std::string & name : names) {
    if (!joined.empty()) {
      joined += ", ";
    }
    joined += name;
  }
  return joined;
}

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

/// Reinterpret an [r, g, b, a] parameter as a color. The parameter is validated to hold exactly
/// four entries in [0.0, 1.0].
std_msgs::msg::ColorRGBA colorFromArray(const std::vector<double> & rgba)
{
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(rgba[0]);
  color.g = static_cast<float>(rgba[1]);
  color.b = static_cast<float>(rgba[2]);
  color.a = static_cast<float>(rgba[3]);
  return color;
}

/// Append a closed LINE_STRIP tracing `footprint` to `markers`, taking every shared field from
/// `prototype`. An empty footprint appends nothing.
/// \param prototype Marker carrying the header, type, scale, and lifetime shared by all outlines.
/// \param footprint Open polygon in the frame of `prototype`'s header.
/// \param marker_namespace Namespace to file the outline under.
/// \param color Stroke color.
/// \param id Marker id, unique within `marker_namespace`.
/// \param markers Array the outline is appended to.
void appendFootprintOutline(
  const visualization_msgs::msg::Marker & prototype,
  const Footprint & footprint,
  const char * marker_namespace,
  const std_msgs::msg::ColorRGBA & color,
  int id,
  visualization_msgs::msg::MarkerArray & markers)
{
  if (footprint.empty()) {
    return;
  }

  visualization_msgs::msg::Marker outline = prototype;
  outline.ns = marker_namespace;
  outline.id = id;
  outline.color = color;
  outline.points.reserve(footprint.size() + 1);
  for (const Point2D & vertex : footprint) {
    geometry_msgs::msg::Point point;
    point.x = vertex.x;
    point.y = vertex.y;
    outline.points.push_back(point);
  }
  // Footprints arrive open; repeating the first vertex closes the outline.
  outline.points.push_back(outline.points.front());

  markers.markers.push_back(std::move(outline));
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

  footprint_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("projected_footprints", MARKER_QUEUE_DEPTH);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("projected_path", MARKER_QUEUE_DEPTH);

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
  footprint_marker_pub_->on_activate();
  path_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "activated");
  return CallbackReturn::SUCCESS;
}

ArticulatedProjectorNode::CallbackReturn ArticulatedProjectorNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  footprint_marker_pub_->on_deactivate();
  path_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "deactivated");
  return CallbackReturn::SUCCESS;
}

ArticulatedProjectorNode::CallbackReturn ArticulatedProjectorNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  joint_state_sub_.reset();
  cmd_vel_sub_.reset();
  footprint_marker_pub_.reset();
  path_pub_.reset();
  projector_.reset();
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    articulation_angle_rad_ = 0.0;
    last_projection_.clear();
    articulation_angle_seen_.store(false);
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
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      LOG_THROTTLE_MS,
      "joint '%s' is not in this JointState; it names [%s]",
      params_.articulation_joint_name.c_str(),
      joinNames(msg.name).c_str());
    return;
  }

  // position[] is allowed to be shorter than name[]: a joint can be reported with velocity/effort only.
  const size_t index = static_cast<size_t>(std::distance(msg.name.begin(), joint));
  if (index >= msg.position.size()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "joint '%s' carries no position", params_.articulation_joint_name.c_str());
    return;
  }

  const double angle_rad = msg.position[index];
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    articulation_angle_rad_ = angle_rad;
  }

  if (!articulation_angle_seen_.exchange(true)) {
    RCLCPP_INFO(
      get_logger(),
      "first articulation angle from joint '%s' (index %zu of %zu): %.4f rad (%.2f deg)",
      params_.articulation_joint_name.c_str(),
      index,
      msg.name.size(),
      angle_rad,
      angle_rad * RAD_TO_DEG);
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    LOG_THROTTLE_MS,
    "articulation angle: %.4f rad (%.2f deg)",
    angle_rad,
    angle_rad * RAD_TO_DEG);
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

  std::unique_ptr<visualization_msgs::msg::MarkerArray> markers;
  std::unique_ptr<nav_msgs::msg::Path> path;
  double measured_angle_rad = 0.0;
  double final_angle_rad = 0.0;
  double final_yaw_rate_rad_s = 0.0;
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    last_projection_ = projector_->project(
      params_.projection.horizon_s,
      params_.projection.time_step_s,
      Pose2D{0.0, 0.0, 0.0},
      articulation_angle_rad_,
      commanded.articulation_angle_rad,
      params_.projector.articulation_rate_rad_s,
      msg.twist.linear.x);
    markers = produceProjectedFootprintMarkers(last_projection_);
    path = produceProjectedPath(last_projection_);
    measured_angle_rad = articulation_angle_rad_;
    if (!last_projection_.empty()) {
      final_angle_rad = last_projection_.back().articulation_angle_rad;
      final_yaw_rate_rad_s = last_projection_.back().angular_velocity_rad_s;
    }
  }

  if (!articulation_angle_seen_.load()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      LOG_THROTTLE_MS,
      "projecting from articulation angle 0.0 rad: no JointState naming '%s' has arrived yet",
      params_.articulation_joint_name.c_str());
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    LOG_THROTTLE_MS,
    "cmd_vel v=%.3f m/s w=%.3f rad/s -> articulation measured %.4f, target %.4f, reached %.4f rad "
    "after %.2fs at %.3f rad/s (final yaw rate %.4f rad/s)",
    msg.twist.linear.x,
    msg.twist.angular.z,
    measured_angle_rad,
    commanded.articulation_angle_rad,
    final_angle_rad,
    params_.projection.horizon_s,
    params_.projector.articulation_rate_rad_s,
    final_yaw_rate_rad_s);

  footprint_marker_pub_->publish(std::move(markers));
  path_pub_->publish(std::move(path));
}

std::unique_ptr<visualization_msgs::msg::MarkerArray> ArticulatedProjectorNode::produceProjectedFootprintMarkers(
  const std::vector<ArticulatedProjectedState> & projection) const
{
  auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
  markers->markers.reserve(2 * projection.size() + 1);

  visualization_msgs::msg::Marker clear_previous;
  clear_previous.action = visualization_msgs::msg::Marker::DELETEALL;
  markers->markers.push_back(clear_previous);

  visualization_msgs::msg::Marker prototype;
  prototype.header.frame_id = params_.visualization.frame_id;
  prototype.header.stamp = now();
  prototype.type = visualization_msgs::msg::Marker::LINE_STRIP;
  prototype.action = visualization_msgs::msg::Marker::ADD;
  prototype.pose.position.z = MARKER_Z_OFFSET_M;
  prototype.pose.orientation.w = 1.0;
  prototype.scale.x = params_.visualization.line_width_m;
  prototype.lifetime = rclcpp::Duration::from_seconds(params_.visualization.marker_lifetime_s);

  const std_msgs::msg::ColorRGBA front_color = colorFromArray(params_.visualization.front_color);
  const std_msgs::msg::ColorRGBA rear_color = colorFromArray(params_.visualization.rear_color);

  // The projector emits footprints already in the projection frame, so each outline carries those
  // points directly and its marker pose stays at the origin.
  int marker_id = 0;
  for (const ArticulatedProjectedState & sample : projection) {
    appendFootprintOutline(prototype, sample.front_footprint, FRONT_MARKER_NAMESPACE, front_color, marker_id, *markers);
    appendFootprintOutline(prototype, sample.rear_footprint, REAR_MARKER_NAMESPACE, rear_color, marker_id, *markers);
    ++marker_id;
  }

  return markers;
}

std::unique_ptr<nav_msgs::msg::Path> ArticulatedProjectorNode::produceProjectedPath(
  const std::vector<ArticulatedProjectedState> & projection) const
{
  auto path = std::make_unique<nav_msgs::msg::Path>();
  path->header.frame_id = params_.visualization.frame_id;
  path->header.stamp = now();
  path->poses.reserve(projection.size());

  for (const ArticulatedProjectedState & sample : projection) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path->header;
    pose.pose.position.x = sample.pose.x;
    pose.pose.position.y = sample.pose.y;
    pose.pose.orientation.z = std::sin(sample.pose.theta / 2.0);
    pose.pose.orientation.w = std::cos(sample.pose.theta / 2.0);
    path->poses.push_back(pose);
  }

  return path;
}

}  // namespace polymath::kinematics::ros2
