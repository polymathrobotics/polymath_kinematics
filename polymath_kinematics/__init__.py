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

import math

from polymath_kinematics_cpp import (
    ArticulatedAxleVelocities,
    ArticulatedModel,
    ArticulatedVehicleState,
    BicycleBodyVelocity,
    BicycleModel,
    BicycleSteeringState,
    DifferentialDriveBodyVelocity,
    DifferentialDriveModel,
    DifferentialDriveWheelVelocities,
)

__all__ = [
    'ArticulatedAxleVelocities',
    'ArticulatedModel',
    'ArticulatedVehicleState',
    'BicycleBodyVelocity',
    'BicycleModel',
    'BicycleSteeringState',
    'DifferentialDriveBodyVelocity',
    'DifferentialDriveModel',
    'DifferentialDriveWheelVelocities',
    'Pose2D',
    'Twist2D',
    'normalize_angle',
    'transform_pose',
]


class Pose2D:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta


class Twist2D:
    def __init__(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        self.linear_x = linear_x
        self.linear_y = linear_y
        self.angular_z = angular_z


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def transform_pose(pose, transform):
    """Transform a pose by a relative transform"""
    result = Pose2D()
    result.x = pose.x + transform.x * math.cos(pose.theta) - transform.y * math.sin(pose.theta)
    result.y = pose.y + transform.x * math.sin(pose.theta) + transform.y * math.cos(pose.theta)
    result.theta = normalize_angle(pose.theta + transform.theta)
    return result
