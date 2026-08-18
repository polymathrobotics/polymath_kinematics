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

from polymath_kinematics_cpp import (
    ArticulatedAxleVelocities,
    ArticulatedModel,
    ArticulatedProjectedState,
    ArticulatedProjector,
    ArticulatedVehicleState,
    AxleReference,
    BicycleBodyVelocity,
    BicycleModel,
    BicycleProjectedState,
    BicycleProjector,
    BicycleSteeringState,
    DifferentialDriveBodyVelocity,
    DifferentialDriveModel,
    DifferentialDriveProjectedState,
    DifferentialDriveProjector,
    DifferentialDriveWheelVelocities,
    Point2D,
    Pose2D,
    rectangle_footprint,
    transform_footprint,
)

# The C++ Footprint is std::vector<Point2D>, which pybind11 exposes as a plain list of Point2D.
# Alias it so annotations can name the concept.
Footprint = list[Point2D]

__all__ = [
    'ArticulatedAxleVelocities',
    'ArticulatedModel',
    'ArticulatedProjectedState',
    'ArticulatedProjector',
    'ArticulatedVehicleState',
    'AxleReference',
    'BicycleBodyVelocity',
    'BicycleModel',
    'BicycleProjectedState',
    'BicycleProjector',
    'BicycleSteeringState',
    'DifferentialDriveBodyVelocity',
    'DifferentialDriveModel',
    'DifferentialDriveProjectedState',
    'DifferentialDriveProjector',
    'DifferentialDriveWheelVelocities',
    'Footprint',
    'Point2D',
    'Pose2D',
    'rectangle_footprint',
    'transform_footprint',
]
