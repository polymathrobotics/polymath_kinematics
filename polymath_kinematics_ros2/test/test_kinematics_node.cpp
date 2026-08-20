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

/// Spin `node` until `predicate` holds or the budget runs out, so a test never blocks forever on a
/// message that is not coming.
/// \return True if the predicate held before the budget expired.
template <typename PredicateT>
bool spinUntil(const std::shared_ptr<ArticulatedProjectorNode> & node, PredicateT predicate)
{
  constexpr int MAX_SPINS = 200;
  for (int spin = 0; spin < MAX_SPINS; ++spin) {
    if (predicate()) {
      return true;
    }
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
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
