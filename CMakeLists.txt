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
project(polymath_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_link_options(-Wl,-no-undefined)
endif()

# Ubuntu detection (override with -DBUILD_JAMMY=ON/OFF)
if(NOT DEFINED BUILD_JAMMY)
  set(BUILD_JAMMY OFF)
  if(EXISTS "/etc/os-release")
    file(READ "/etc/os-release" _OS_RELEASE)
    string(REGEX MATCH "VERSION_CODENAME=([^\n\r]+)" _match "${_OS_RELEASE}")
    if(CMAKE_MATCH_1)
      set(_UBUNTU_CODENAME "${CMAKE_MATCH_1}")
      string(TOLOWER "${_UBUNTU_CODENAME}" _UBUNTU_CODENAME)
      if(_UBUNTU_CODENAME STREQUAL "jammy")
        set(BUILD_JAMMY ON)
        message(STATUS "Ubuntu=jammy (22.04) -> Catch2 v2")
      else()
        set(BUILD_JAMMY OFF)
        message(STATUS "Ubuntu codename='${_UBUNTU_CODENAME}' -> defaulting to Catch2 v3")
      endif()
    endif()
  endif()
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# C++ library with all kinematic models
add_library(polymath_kinematics SHARED
  src/articulated_model.cpp
  src/differential_drive_model.cpp
  src/bicycle_model.cpp
)
target_include_directories(polymath_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Python bindings
set(PYBIND_TARGET polymath_kinematics_cpp)
pybind11_add_module(${PYBIND_TARGET}
  src/kinematics_pybind.cpp
)
set_target_properties(${PYBIND_TARGET} PROPERTIES LINK_OPTIONS "")
target_link_libraries(${PYBIND_TARGET} PRIVATE polymath_kinematics)
_ament_cmake_python_register_environment_hook()
install(TARGETS ${PYBIND_TARGET} DESTINATION "${PYTHON_INSTALL_DIR}")

# Install C++ library
install(
  TARGETS polymath_kinematics
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

# Python module
ament_python_install_package(${PROJECT_NAME})

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  include(CTest)

  if(BUILD_JAMMY)
    find_package(Catch2 2 REQUIRED)
  else()
    find_package(Catch2 3 REQUIRED)
  endif()
  include(Catch OPTIONAL)

  function(add_kinematics_catch2_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_link_libraries(${TEST_NAME} PRIVATE polymath_kinematics Catch2::Catch2WithMain)
    target_include_directories(${TEST_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/test)
    if(COMMAND catch_discover_tests)
      catch_discover_tests(${TEST_NAME})
    else()
      add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
    endif()
  endfunction()

  add_kinematics_catch2_test(test_differential_drive test/test_differential_drive.cpp)
  add_kinematics_catch2_test(test_bicycle_model test/test_bicycle_model.cpp)
  add_kinematics_catch2_test(test_articulated_model test/test_articulated_model.cpp)
  ament_add_pytest_test(test_python_bindings test)
endif()

# Export targets
ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(polymath_kinematics)

ament_package()
