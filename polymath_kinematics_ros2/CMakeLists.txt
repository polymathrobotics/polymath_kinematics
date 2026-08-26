# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(polymath_kinematics_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_link_options(-Wl,-no-undefined)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_library(
  articulated_projector_params
  src/articulated_projector.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/articulated_projector_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
    articulated_projector_params
    polymath_kinematics::polymath_kinematics
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${geometry_msgs_TARGETS}
    ${lifecycle_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
  PRIVATE
    magic_enum::magic_enum
    rclcpp_components::component
)

# Upstream rclcpp_components, not polymath_core's rclcpp_lifecycle_components wrapper: that
# package lives in polymath_core and is unavailable when this repo builds standalone in its own CI.
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "polymath::kinematics::ros2::ArticulatedProjector"
  EXECUTABLE articulated_projector
)

install(
  TARGETS ${PROJECT_NAME} articulated_projector articulated_projector_params
  EXPORT ${PROJECT_NAME}_TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  EXPORT ${PROJECT_NAME}_TARGETS
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)
install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  include(CTest)

  # Jammy (22.04) ships Catch2 v2; every later Ubuntu ships v3. Override with -DBUILD_JAMMY=ON/OFF.
  if(NOT DEFINED BUILD_JAMMY)
    set(BUILD_JAMMY OFF)
    if(EXISTS "/etc/os-release")
      file(READ "/etc/os-release" OS_RELEASE)
      string(REGEX MATCH "VERSION_CODENAME=([^\n\r]+)" MATCHED "${OS_RELEASE}")
      if(CMAKE_MATCH_1)
        string(TOLOWER "${CMAKE_MATCH_1}" UBUNTU_CODENAME)
        if(UBUNTU_CODENAME STREQUAL "jammy")
          set(BUILD_JAMMY ON)
        endif()
      endif()
    endif()
  endif()

  if(BUILD_JAMMY)
    find_package(Catch2 2 REQUIRED)
  else()
    find_package(Catch2 3 REQUIRED)
  endif()
  include(Catch OPTIONAL)

  # test/catch2_compat.hpp bridges the v2/v3 header and Approx differences.
  add_executable(test_kinematics_node test/test_kinematics_node.cpp)
  target_link_libraries(test_kinematics_node PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain)
  target_include_directories(test_kinematics_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
  if(COMMAND catch_discover_tests)
    # PRE_TEST enumerates at ctest time, not during the build where a stale installed .so can win.
    catch_discover_tests(test_kinematics_node DISCOVERY_MODE PRE_TEST)
  else()
    add_test(NAME test_kinematics_node COMMAND test_kinematics_node)
  endif()
endif()

ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)
ament_package()
