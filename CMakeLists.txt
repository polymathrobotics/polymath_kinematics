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

# Ubuntu codename detection -> Catch2 v2 (Jammy) vs v3 (everything else).
# Mirrors socketcan_adapter so the Catch2 version is picked deterministically
# per distro instead of best-effort QUIET probing.
if(NOT DEFINED BUILD_JAMMY)
  set(BUILD_JAMMY OFF)
  if(EXISTS "/etc/os-release")
    file(READ "/etc/os-release" _OS_RELEASE)
    string(REGEX MATCH "VERSION_CODENAME=([^\n\r]+)" _match "${_OS_RELEASE}")
    if(CMAKE_MATCH_1)
      string(TOLOWER "${CMAKE_MATCH_1}" _UBUNTU_CODENAME)
      if(_UBUNTU_CODENAME STREQUAL "jammy")
        set(BUILD_JAMMY ON)
        message(STATUS "Ubuntu=jammy (22.04) -> Catch2 v2")
      else()
        message(STATUS "Ubuntu codename='${_UBUNTU_CODENAME}' -> defaulting to Catch2 v3")
      endif()
    endif()
  endif()
endif()

# ament_cmake is only present under colcon. Absent under pip wheel builds,
# which is fine: the QUIET probe simply leaves ament_cmake_FOUND false.
find_package(ament_cmake QUIET)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

# Library target name must stay `polymath_kinematics`; downstream packages link
# against it by that name. SHARED under ROS2 so consumers pick up the .so via
# ament_export_targets; STATIC under wheel builds so the pybind11 module is
# self-contained and the wheel ships a single shared object.
set(_pk_lib_sources
  src/articulated_model.cpp
  src/articulated_projector.cpp
  src/bicycle_model.cpp
  src/bicycle_projector.cpp
  src/differential_drive_model.cpp
  src/differential_drive_projector.cpp
)
if(ament_cmake_FOUND)
  add_library(polymath_kinematics SHARED ${_pk_lib_sources})
else()
  add_library(polymath_kinematics STATIC ${_pk_lib_sources})
endif()
target_include_directories(polymath_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(polymath_kinematics PROPERTIES POSITION_INDEPENDENT_CODE ON)

pybind11_add_module(polymath_kinematics_cpp src/kinematics_pybind.cpp)
target_link_libraries(polymath_kinematics_cpp PRIVATE polymath_kinematics)

if(ament_cmake_FOUND)
  find_package(ament_cmake_python REQUIRED)

  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS polymath_kinematics
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)

  ament_python_install_package(polymath_kinematics)
  # `polymath_kinematics_cpp` is a top-level module imported by polymath_kinematics/__init__.py,
  # not a submodule. Install it as a sibling of the Python package, matching the wheel layout.
  install(TARGETS polymath_kinematics_cpp
    DESTINATION "${PYTHON_INSTALL_DIR}"
  )
else()
  # Wheel build: scikit-build-core installs the extension at the wheel root so
  # `from polymath_kinematics_cpp import ...` keeps working.
  install(TARGETS polymath_kinematics_cpp DESTINATION .)
endif()

if(BUILD_TESTING)
  include(CTest)
  if(BUILD_JAMMY)
    find_package(Catch2 2 REQUIRED)
  else()
    find_package(Catch2 3 REQUIRED)
  endif()
  include(Catch OPTIONAL)

  set(_pk_tests
    test_bicycle_model
    test_articulated_model
    test_differential_drive
    test_bicycle_projector
    test_articulated_projector
    test_differential_drive_projector
  )
  foreach(tgt IN LISTS _pk_tests)
    add_executable(${tgt} test/${tgt}.cpp)
    target_link_libraries(${tgt} PRIVATE polymath_kinematics Catch2::Catch2WithMain)
    target_include_directories(${tgt} PRIVATE test)
    if(COMMAND catch_discover_tests)
      catch_discover_tests(${tgt})
    else()
      add_test(NAME ${tgt} COMMAND ${tgt})
    endif()
  endforeach()

  if(ament_cmake_FOUND)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_python_bindings test/test_python_bindings.py
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    ament_add_pytest_test(test_explorer test/test_explorer.py
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endif()
endif()

if(ament_cmake_FOUND)
  ament_package()
endif()
