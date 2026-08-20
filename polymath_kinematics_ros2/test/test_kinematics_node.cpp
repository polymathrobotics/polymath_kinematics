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

#include <memory>
#include <string>

#include "catch2_compat.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "polymath_kinematics_ros2/kinematics_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

using polymath::kinematics_ros2::KinematicsNode;

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

}  // namespace

TEST_CASE("KinematicsNode walks the full lifecycle", "[kinematics_node]")
{
  const RclcppFixture fixture;
  auto node = std::make_shared<KinematicsNode>(rclcpp::NodeOptions());

  REQUIRE(std::string("kinematics_node") == std::string(node->get_name()));

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
  auto node = std::make_shared<KinematicsNode>(rclcpp::NodeOptions());

  const rclcpp_lifecycle::State state = node->get_current_state();
  REQUIRE(KinematicsNode::CallbackReturn::SUCCESS == node->on_configure(state));
  REQUIRE(KinematicsNode::CallbackReturn::SUCCESS == node->on_activate(state));
  REQUIRE(KinematicsNode::CallbackReturn::SUCCESS == node->on_deactivate(state));
  REQUIRE(KinematicsNode::CallbackReturn::SUCCESS == node->on_cleanup(state));
}
