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

#include "polymath_kinematics/articulated_model.hpp"
#include "polymath_kinematics/articulated_projector.hpp"
#include "polymath_kinematics/bicycle_model.hpp"
#include "polymath_kinematics/bicycle_projector.hpp"
#include "polymath_kinematics/differential_drive_model.hpp"
#include "polymath_kinematics/pose2d.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;

namespace polymath::kinematics
{

PYBIND11_MODULE(polymath_kinematics_cpp, m)
{
  m.doc() = "Python bindings for polymath kinematics library.";

  // Articulated model bindings
  py::class_<ArticulatedVehicleState>(m, "ArticulatedVehicleState")
    .def(py::init<>())
    .def_readwrite("articulation_angle_rad", &ArticulatedVehicleState::articulation_angle_rad)
    .def_readwrite("linear_velocity_m_s", &ArticulatedVehicleState::linear_velocity_m_s)
    .def_readwrite("front_right_wheel_speed_rad_s", &ArticulatedVehicleState::front_right_wheel_speed_rad_s)
    .def_readwrite("front_left_wheel_speed_rad_s", &ArticulatedVehicleState::front_left_wheel_speed_rad_s)
    .def_readwrite("rear_right_wheel_speed_rad_s", &ArticulatedVehicleState::rear_right_wheel_speed_rad_s)
    .def_readwrite("rear_left_wheel_speed_rad_s", &ArticulatedVehicleState::rear_left_wheel_speed_rad_s)
    .def_readwrite("front_axle_turning_radius_m", &ArticulatedVehicleState::front_axle_turning_radius_m)
    .def_readwrite("rear_axle_turning_radius_m", &ArticulatedVehicleState::rear_axle_turning_radius_m);

  py::class_<ArticulatedAxleVelocities>(m, "ArticulatedAxleVelocities")
    .def(py::init<>())
    .def_readwrite("linear_velocity_m_s", &ArticulatedAxleVelocities::linear_velocity_m_s)
    .def_readwrite("front_axle_turning_velocity_rad_s", &ArticulatedAxleVelocities::front_axle_turning_velocity_rad_s)
    .def_readwrite("rear_axle_turning_velocity_rad_s", &ArticulatedAxleVelocities::rear_axle_turning_velocity_rad_s);

  py::class_<ArticulatedModel>(m, "ArticulatedModel")
    .def(
      py::init<double, double, double, double, double, double>(),
      py::arg("articulation_to_front_axle_m"),
      py::arg("articulation_to_rear_axle_m"),
      py::arg("front_track_width_m"),
      py::arg("rear_track_width_m"),
      py::arg("front_wheel_radius_m"),
      py::arg("rear_wheel_radius_m"))
    .def(
      "body_velocity_to_vehicle_state",
      py::overload_cast<double, double, double>(&ArticulatedModel::bodyVelocityToVehicleState),
      py::arg("linear_velocity_m_s"),
      py::arg("angular_velocity_rad_s"),
      py::arg("articulation_turning_velocity_rad_s") = 0.0)
    .def(
      "articulation_to_axle_velocities",
      py::overload_cast<double, double, double>(&ArticulatedModel::articulationToAxleVelocities),
      py::arg("linear_velocity_m_s"),
      py::arg("articulation_angle_rad"),
      py::arg("articulation_turning_velocity_rad_s") = 0.0)
    .def_property_readonly("articulation_to_front_axle", &ArticulatedModel::get_articulation_to_front_axle_m)
    .def_property_readonly("articulation_to_rear_axle", &ArticulatedModel::get_articulation_to_rear_axle_m)
    .def_property_readonly("front_track_width", &ArticulatedModel::get_front_track_width_m)
    .def_property_readonly("rear_track_width", &ArticulatedModel::get_rear_track_width_m)
    .def_property_readonly("front_wheel_radius", &ArticulatedModel::get_front_wheel_radius_m)
    .def_property_readonly("rear_wheel_radius", &ArticulatedModel::get_rear_wheel_radius_m);

  // Differential drive model bindings
  py::class_<DifferentialDriveWheelVelocities>(m, "DifferentialDriveWheelVelocities")
    .def(py::init<>())
    .def_readwrite("left_wheel_velocity_rad_s", &DifferentialDriveWheelVelocities::left_wheel_velocity_rad_s)
    .def_readwrite("right_wheel_velocity_rad_s", &DifferentialDriveWheelVelocities::right_wheel_velocity_rad_s);

  py::class_<DifferentialDriveBodyVelocity>(m, "DifferentialDriveBodyVelocity")
    .def(py::init<>())
    .def_readwrite("linear_velocity_m_s", &DifferentialDriveBodyVelocity::linear_velocity_m_s)
    .def_readwrite("angular_velocity_rad_s", &DifferentialDriveBodyVelocity::angular_velocity_rad_s);

  py::class_<DifferentialDriveModel>(m, "DifferentialDriveModel")
    .def(py::init<double, double>(), py::arg("wheel_radius_m"), py::arg("track_width_m"))
    .def(
      "wheel_velocities_to_body_velocity",
      &DifferentialDriveModel::wheelVelocitiesToBodyVelocity,
      py::arg("left_wheel_vel"),
      py::arg("right_wheel_vel"))
    .def(
      "body_velocity_to_wheel_velocities",
      &DifferentialDriveModel::bodyVelocityToWheelVelocities,
      py::arg("linear_vel"),
      py::arg("angular_vel"))
    .def_property_readonly("wheel_radius", &DifferentialDriveModel::get_wheel_radius_m)
    .def_property_readonly("track_width", &DifferentialDriveModel::get_track_width_m);

  // Bicycle model bindings
  py::class_<BicycleSteeringState>(m, "BicycleSteeringState")
    .def(py::init<>())
    .def_readwrite("velocity_m_s", &BicycleSteeringState::velocity_m_s)
    .def_readwrite("steering_angle_rad", &BicycleSteeringState::steering_angle_rad)
    .def_readwrite("turning_radius_m", &BicycleSteeringState::turning_radius_m)
    .def_readwrite("front_right_wheel_rad_s", &BicycleSteeringState::front_right_wheel_rad_s)
    .def_readwrite("front_left_wheel_rad_s", &BicycleSteeringState::front_left_wheel_rad_s)
    .def_readwrite("rear_right_wheel_rad_s", &BicycleSteeringState::rear_right_wheel_rad_s)
    .def_readwrite("rear_left_wheel_rad_s", &BicycleSteeringState::rear_left_wheel_rad_s);

  py::class_<BicycleBodyVelocity>(m, "BicycleBodyVelocity")
    .def(py::init<>())
    .def_readwrite("linear_velocity_m_s", &BicycleBodyVelocity::linear_velocity_m_s)
    .def_readwrite("angular_velocity_rad_s", &BicycleBodyVelocity::angular_velocity_rad_s);

  py::class_<BicycleModel>(m, "BicycleModel")
    .def(
      py::init<double, double, double>(), py::arg("wheelbase_m"), py::arg("track_width_m"), py::arg("wheel_radius_m"))
    .def(
      "steering_to_body_velocity",
      &BicycleModel::steeringToBodyVelocity,
      py::arg("velocity"),
      py::arg("steering_angle"))
    .def(
      "body_velocity_to_steering",
      &BicycleModel::bodyVelocityToSteering,
      py::arg("linear_velocity"),
      py::arg("angular_velocity"))
    .def("turning_radius", &BicycleModel::turningRadius, py::arg("steering_angle"))
    .def("steering_angle_from_radius", &BicycleModel::steeringAngleFromRadius, py::arg("radius"))
    .def_property_readonly("wheelbase", &BicycleModel::get_wheelbase_m)
    .def_property_readonly("track_width", &BicycleModel::get_track_width_m)
    .def_property_readonly("wheel_radius", &BicycleModel::get_wheel_radius_m);

  // Shared Pose2D
  py::class_<Pose2D>(m, "Pose2D")
    .def(py::init<>())
    .def(
      py::init([](double x, double y, double theta) { return Pose2D{x, y, theta}; }),
      py::arg("x") = 0.0,
      py::arg("y") = 0.0,
      py::arg("theta") = 0.0)
    .def_readwrite("x", &Pose2D::x)
    .def_readwrite("y", &Pose2D::y)
    .def_readwrite("theta", &Pose2D::theta);

  // Bicycle projector bindings
  py::class_<BicycleProjectedState>(m, "BicycleProjectedState")
    .def(py::init<>())
    .def_readwrite("time_s", &BicycleProjectedState::time_s)
    .def_readwrite("pose", &BicycleProjectedState::pose)
    .def_readwrite("steering_angle_rad", &BicycleProjectedState::steering_angle_rad)
    .def_readwrite("linear_velocity_m_s", &BicycleProjectedState::linear_velocity_m_s)
    .def_readwrite("angular_velocity_rad_s", &BicycleProjectedState::angular_velocity_rad_s)
    .def_readwrite("steering_state", &BicycleProjectedState::steering_state);

  py::class_<BicycleProjector>(m, "BicycleProjector")
    .def(
      py::init<BicycleModel, double, double>(),
      py::arg("model"),
      py::arg("min_steering_angle_rad"),
      py::arg("max_steering_angle_rad"))
    .def(
      "step",
      &BicycleProjector::step,
      py::arg("dt_s"),
      py::arg("current_pose"),
      py::arg("current_steering_angle_rad"),
      py::arg("target_steering_angle_rad"),
      py::arg("steering_rate_rad_s"),
      py::arg("linear_velocity_m_s"))
    .def(
      "project",
      &BicycleProjector::project,
      py::arg("horizon_s"),
      py::arg("dt_s"),
      py::arg("initial_pose"),
      py::arg("initial_steering_angle_rad"),
      py::arg("target_steering_angle_rad"),
      py::arg("steering_rate_rad_s"),
      py::arg("linear_velocity_m_s"))
    .def_property_readonly("model", &BicycleProjector::get_model, py::return_value_policy::reference_internal)
    .def_property_readonly("min_steering_angle_rad", &BicycleProjector::get_min_steering_angle_rad)
    .def_property_readonly("max_steering_angle_rad", &BicycleProjector::get_max_steering_angle_rad);

  // Articulated projector bindings
  py::class_<ArticulatedProjectedState>(m, "ArticulatedProjectedState")
    .def(py::init<>())
    .def_readwrite("time_s", &ArticulatedProjectedState::time_s)
    .def_readwrite("pose", &ArticulatedProjectedState::pose)
    .def_readwrite("articulation_angle_rad", &ArticulatedProjectedState::articulation_angle_rad)
    .def_readwrite("linear_velocity_m_s", &ArticulatedProjectedState::linear_velocity_m_s)
    .def_readwrite("angular_velocity_rad_s", &ArticulatedProjectedState::angular_velocity_rad_s)
    .def_readwrite("vehicle_state", &ArticulatedProjectedState::vehicle_state);

  py::class_<ArticulatedProjector>(m, "ArticulatedProjector")
    .def(
      py::init<ArticulatedModel, double, double>(),
      py::arg("model"),
      py::arg("min_articulation_angle_rad"),
      py::arg("max_articulation_angle_rad"))
    .def(
      "step",
      &ArticulatedProjector::step,
      py::arg("dt_s"),
      py::arg("current_pose"),
      py::arg("current_articulation_angle_rad"),
      py::arg("target_articulation_angle_rad"),
      py::arg("articulation_rate_rad_s"),
      py::arg("linear_velocity_m_s"))
    .def(
      "project",
      &ArticulatedProjector::project,
      py::arg("horizon_s"),
      py::arg("dt_s"),
      py::arg("initial_pose"),
      py::arg("initial_articulation_angle_rad"),
      py::arg("target_articulation_angle_rad"),
      py::arg("articulation_rate_rad_s"),
      py::arg("linear_velocity_m_s"))
    .def_property_readonly("model", &ArticulatedProjector::get_model, py::return_value_policy::reference_internal)
    .def_property_readonly("min_articulation_angle_rad", &ArticulatedProjector::get_min_articulation_angle_rad)
    .def_property_readonly("max_articulation_angle_rad", &ArticulatedProjector::get_max_articulation_angle_rad);
}

}  // namespace polymath::kinematics
