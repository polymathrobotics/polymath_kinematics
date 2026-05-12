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

cmake_minimum_required(VERSION 3.15)
project(polymath_kinematics CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# Core static library — the kinematic models and projectors live here.
add_library(polymath_kinematics_core STATIC
  src/articulated_model.cpp
  src/articulated_projector.cpp
  src/bicycle_model.cpp
  src/bicycle_projector.cpp
  src/differential_drive_model.cpp
)
target_include_directories(polymath_kinematics_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
set_target_properties(polymath_kinematics_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Python extension — exposes everything as `polymath_kinematics_cpp`.
pybind11_add_module(polymath_kinematics_cpp src/kinematics_pybind.cpp)
target_link_libraries(polymath_kinematics_cpp PRIVATE polymath_kinematics_core)

# Install the extension module at the wheel root so `from polymath_kinematics_cpp import ...` works.
install(TARGETS polymath_kinematics_cpp DESTINATION .)

# ----------------------------------------------------------------------
# Optional C++ tests (off by default; build with -DBUILD_TESTING=ON).
# Not part of the wheel build. Run with: cmake --build && ctest.
# ----------------------------------------------------------------------
option(BUILD_TESTING "Build C++ Catch2 tests" OFF)

if(BUILD_TESTING)
  include(CTest)

  find_package(Catch2 3 QUIET)
  if(NOT Catch2_FOUND)
    find_package(Catch2 2 REQUIRED)
  endif()

  set(_pk_tests
    test_bicycle_model
    test_articulated_model
    test_differential_drive
    test_bicycle_projector
    test_articulated_projector
  )
  foreach(tgt IN LISTS _pk_tests)
    add_executable(${tgt} test/${tgt}.cpp)
    target_link_libraries(${tgt} PRIVATE polymath_kinematics_core Catch2::Catch2WithMain)
    target_include_directories(${tgt} PRIVATE test)
    add_test(NAME ${tgt} COMMAND ${tgt})
  endforeach()
endif()
