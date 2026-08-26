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

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "catch2_compat.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "polymath_kinematics_ros2/articulated_projector_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

using polymath::kinematics::ros2::ArticulatedProjectorNode;

/// Brings rclcpp up for the duration of a test case and tears it down again, so the suite can be
/// run repeatedly in one process without leaking context state.
class RclcppFixture
{
public:
  RclcppFixture()
  {
    rclcpp::init(0, nullptr);
  }

  ~RclcppFixture()
  {
    rclcpp::shutdown();
  }

  RclcppFixture(const RclcppFixture &) = delete;
  RclcppFixture & operator=(const RclcppFixture &) = delete;
};

/// Spin every node in `nodes` until `predicate` holds or the budget runs out, so a test never
/// blocks forever on a message that is not coming.
/// \return True if the predicate held before the budget expired.
template <typename PredicateT>
bool spinUntil(const std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> & nodes, PredicateT predicate)
{
  constexpr int MAX_SPINS = 200;
  for (int spin = 0; spin < MAX_SPINS; ++spin) {
    if (predicate()) {
      return true;
    }
    for (const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & base : nodes) {
      rclcpp::spin_some(base);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

/// Spin the node under test alone.
/// \return True if the predicate held before the budget expired.
template <typename PredicateT>
bool spinUntil(const std::shared_ptr<ArticulatedProjectorNode> & node, PredicateT predicate)
{
  return spinUntil({node->get_node_base_interface()}, predicate);
}

/// Build a JointState naming a decoy joint ahead of the articulation joint, so a test that passes
/// cannot be passing by reading index 0.
sensor_msgs::msg::JointState makeJointState(const std::string & articulation_joint_name, double angle_rad)
{
  sensor_msgs::msg::JointState msg;
  msg.name = {"some_other_joint", articulation_joint_name};
  msg.position = {0.1, angle_rad};
  return msg;
}

}  // namespace

TEST_CASE("KinematicsNode walks the full lifecycle", "[kinematics_node]")
{
  const RclcppFixture fixture;
  auto node = std::make_shared<ArticulatedProjectorNode>(rclcpp::NodeOptions());

  REQUIRE(std::string("articulated_projector") == std::string(node->get_name()));

  const rclcpp_lifecycle::State unconfigured = node->get_current_state();
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED == unconfigured.id());

  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE == node->configure().id());
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE == node->activate().id());
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE == node->deactivate().id());
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED == node->cleanup().id());
}

TEST_CASE("KinematicsNode transition callbacks report success", "[kinematics_node]")
{
  const RclcppFixture fixture;
  auto node = std::make_shared<ArticulatedProjectorNode>(rclcpp::NodeOptions());

  const rclcpp_lifecycle::State state = node->get_current_state();
  REQUIRE(ArticulatedProjectorNode::CallbackReturn::SUCCESS == node->on_configure(state));
  REQUIRE(ArticulatedProjectorNode::CallbackReturn::SUCCESS == node->on_activate(state));
  REQUIRE(ArticulatedProjectorNode::CallbackReturn::SUCCESS == node->on_deactivate(state));
  REQUIRE(ArticulatedProjectorNode::CallbackReturn::SUCCESS == node->on_cleanup(state));
}

TEST_CASE("KinematicsNode latches the articulation angle from the named joint", "[kinematics_node]")
{
  const RclcppFixture fixture;
  auto node = std::make_shared<ArticulatedProjectorNode>(rclcpp::NodeOptions());
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE == node->configure().id());
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE == node->activate().id());

  const std::string joint_name = node->get_parameter("articulation_joint_name").as_string();
  auto publisher_node = std::make_shared<rclcpp::Node>("joint_state_publisher");
  auto publisher = publisher_node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

  publisher->publish(makeJointState(joint_name, 0.25));
  REQUIRE(spinUntil(node, [&node] { return 0.0 != node->getArticulationAngleRad(); }));
  CHECK(node->getArticulationAngleRad() == Approx(0.25));

  // A message that does not name the articulation joint must leave the latched angle alone.
  sensor_msgs::msg::JointState unrelated;
  unrelated.name = {"some_other_joint"};
  unrelated.position = {1.0};
  publisher->publish(unrelated);
  spinUntil(node, [] { return false; });
  CHECK(node->getArticulationAngleRad() == Approx(0.25));
}

TEST_CASE("KinematicsNode projects a trajectory from the latched angle and cmd_vel", "[kinematics_node]")
{
  const RclcppFixture fixture;
  auto node = std::make_shared<ArticulatedProjectorNode>(rclcpp::NodeOptions());
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE == node->configure().id());
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE == node->activate().id());

  const double horizon_s = node->get_parameter("projection.horizon_s").as_double();
  const double time_step_s = node->get_parameter("projection.time_step_s").as_double();
  const std::string joint_name = node->get_parameter("articulation_joint_name").as_string();

  auto publisher_node = std::make_shared<rclcpp::Node>("command_publisher");
  auto joint_publisher = publisher_node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  auto cmd_vel_publisher = publisher_node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);

  joint_publisher->publish(makeJointState(joint_name, 0.2));
  REQUIRE(spinUntil(node, [&node] { return 0.0 != node->getArticulationAngleRad(); }));

  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.x = 1.0;
  command.twist.angular.z = 0.0;
  cmd_vel_publisher->publish(command);
  REQUIRE(spinUntil(node, [&node] { return !node->getLastProjection().empty(); }));

  const std::vector<polymath::kinematics::ArticulatedProjectedState> projection = node->getLastProjection();
  const size_t expected_samples = static_cast<size_t>(std::ceil(horizon_s / time_step_s)) + 1;
  CHECK(expected_samples == projection.size());

  // Element 0 is the initial state: t=0, identity pose, and the angle the joint reported.
  CHECK(projection.front().time_s == Approx(0.0));
  CHECK(projection.front().pose.x == Approx(0.0));
  CHECK(projection.front().articulation_angle_rad == Approx(0.2));

  // A straight-ahead command ramps the articulation back to zero and carries the vehicle forward.
  CHECK(projection.back().time_s == Approx(horizon_s));
  CHECK(projection.back().articulation_angle_rad == Approx(0.0).margin(1e-9));
  CHECK(projection.back().pose.x > projection.front().pose.x);
}

TEST_CASE("KinematicsNode publishes the projected footprints as markers", "[kinematics_node]")
{
  const RclcppFixture fixture;

  // A 2 m x 2 m square about each axle, so every sample contributes a four-vertex outline.
  const std::vector<double> square = {-1.0, -1.0, 1.0, -1.0, 1.0, 1.0, -1.0, 1.0};
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("projector.front_footprint", square),
     rclcpp::Parameter("projector.rear_footprint", square),
     rclcpp::Parameter("visualization.frame_id", std::string("rear_axle"))});

  auto node = std::make_shared<ArticulatedProjectorNode>(options);
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE == node->configure().id());
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE == node->activate().id());

  auto peer_node = std::make_shared<rclcpp::Node>("marker_listener");
  visualization_msgs::msg::MarkerArray received;
  auto marker_subscription = peer_node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "projected_footprints", 1, [&received](const visualization_msgs::msg::MarkerArray & msg) { received = msg; });
  auto cmd_vel_publisher = peer_node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);

  const std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes = {
    node->get_node_base_interface(), peer_node->get_node_base_interface()};

  // Publishing before discovery completes drops the command, and with it the only marker array.
  REQUIRE(spinUntil(nodes, [&marker_subscription, &cmd_vel_publisher] {
    return 0 < marker_subscription->get_publisher_count() && 0 < cmd_vel_publisher->get_subscription_count();
  }));

  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.x = 1.0;
  command.twist.angular.z = 0.2;
  cmd_vel_publisher->publish(command);
  REQUIRE(spinUntil(nodes, [&received] { return !received.markers.empty(); }));

  // One DELETEALL, then a front and a rear outline per projected sample.
  const size_t sample_count = node->getLastProjection().size();
  REQUIRE(0 < sample_count);
  CHECK(2 * sample_count + 1 == received.markers.size());

  CHECK(visualization_msgs::msg::Marker::DELETEALL == received.markers.front().action);

  const double line_width_m = node->get_parameter("visualization.line_width_m").as_double();
  size_t front_outlines = 0;
  size_t rear_outlines = 0;
  for (size_t index = 1; index < received.markers.size(); ++index) {
    const visualization_msgs::msg::Marker & outline = received.markers[index];
    CHECK(visualization_msgs::msg::Marker::ADD == outline.action);
    CHECK(visualization_msgs::msg::Marker::LINE_STRIP == outline.type);
    CHECK(std::string("rear_axle") == outline.header.frame_id);
    CHECK(outline.scale.x == Approx(line_width_m));

    // Four vertices plus the repeat that closes the outline.
    REQUIRE(5 == outline.points.size());
    CHECK(outline.points.front().x == Approx(outline.points.back().x));
    CHECK(outline.points.front().y == Approx(outline.points.back().y));

    if ("projected_front_footprint" == outline.ns) {
      ++front_outlines;
    } else if ("projected_rear_footprint" == outline.ns) {
      ++rear_outlines;
    }
  }
  CHECK(sample_count == front_outlines);
  CHECK(sample_count == rear_outlines);

  // The outlines carry projection-frame points, so a turning command spreads them out.
  const visualization_msgs::msg::Marker & first_rear = received.markers[2];
  const visualization_msgs::msg::Marker & last_rear = received.markers.back();
  CHECK(std::string("projected_rear_footprint") == first_rear.ns);
  CHECK(std::string("projected_rear_footprint") == last_rear.ns);
  CHECK(last_rear.points.front().x > first_rear.points.front().x);
}

TEST_CASE("KinematicsNode publishes no markers for an unset footprint", "[kinematics_node]")
{
  const RclcppFixture fixture;
  auto node = std::make_shared<ArticulatedProjectorNode>(rclcpp::NodeOptions());
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE == node->configure().id());
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE == node->activate().id());

  auto peer_node = std::make_shared<rclcpp::Node>("marker_listener");
  visualization_msgs::msg::MarkerArray received;
  auto marker_subscription = peer_node->create_subscription<visualization_msgs::msg::MarkerArray>(
    "projected_footprints", 1, [&received](const visualization_msgs::msg::MarkerArray & msg) { received = msg; });
  auto cmd_vel_publisher = peer_node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);

  const std::vector<rclcpp::node_interfaces::NodeBaseInterface::SharedPtr> nodes = {
    node->get_node_base_interface(), peer_node->get_node_base_interface()};

  REQUIRE(spinUntil(nodes, [&marker_subscription, &cmd_vel_publisher] {
    return 0 < marker_subscription->get_publisher_count() && 0 < cmd_vel_publisher->get_subscription_count();
  }));

  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.x = 1.0;
  cmd_vel_publisher->publish(command);
  REQUIRE(spinUntil(nodes, [&received] { return !received.markers.empty(); }));

  // The footprint parameters default to empty, leaving only the DELETEALL.
  CHECK(1 == received.markers.size());
  CHECK(visualization_msgs::msg::Marker::DELETEALL == received.markers.front().action);
}

TEST_CASE("KinematicsNode ignores cmd_vel while inactive", "[kinematics_node]")
{
  const RclcppFixture fixture;
  auto node = std::make_shared<ArticulatedProjectorNode>(rclcpp::NodeOptions());
  REQUIRE(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE == node->configure().id());

  auto publisher_node = std::make_shared<rclcpp::Node>("command_publisher");
  auto cmd_vel_publisher = publisher_node->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);

  geometry_msgs::msg::TwistStamped command;
  command.twist.linear.x = 1.0;
  cmd_vel_publisher->publish(command);
  spinUntil(node, [] { return false; });

  CHECK(node->getLastProjection().empty());
}

TEST_CASE("KinematicsNode rejects an empty articulation joint name", "[kinematics_node]")
{
  const RclcppFixture fixture;
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("articulation_joint_name", std::string(""))});

  REQUIRE_THROWS(std::make_shared<ArticulatedProjectorNode>(options));
}
