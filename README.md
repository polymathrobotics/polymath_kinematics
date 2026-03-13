# polymath_kinematics

Kinematic models for differential-drive, bicycle (Ackermann), and articulated vehicles.

## Derivations

Math and equations behind each model:

- [Differential drive](derivations/differential_drive.md) — forward/inverse kinematics for two independently driven wheels
- [Bicycle model (Ackermann)](derivations/bicycle_model.md) — steering-angle kinematics with four-wheel ICR geometry
- [Articulated model](derivations/articulated_model.md) — pivot-joint kinematics for front/rear section vehicles

## C++ usage

```cmake
find_package(polymath_kinematics REQUIRED)
target_link_libraries(my_target PRIVATE polymath_kinematics::polymath_kinematics)
```

```cpp
#include <polymath_kinematics/differential_drive_model.hpp>
#include <polymath_kinematics/bicycle_model.hpp>
#include <polymath_kinematics/articulated_model.hpp>
```

### DifferentialDriveModel

```cpp
polymath::kinematics::DifferentialDriveModel model(0.15, 0.5);  // wheel_radius_m, track_width_m

auto wheels = model.bodyVelocityToWheelVelocities(1.0, 0.3);   // linear m/s, angular rad/s
// wheels.left_wheel_velocity_rad_s, wheels.right_wheel_velocity_rad_s

auto body = model.wheelVelocitiesToBodyVelocity(6.0, 7.0);      // left rad/s, right rad/s
// body.linear_velocity_m_s, body.angular_velocity_rad_s
```

### BicycleModel

```cpp
polymath::kinematics::BicycleModel model(2.7, 1.6, 0.35);  // wheelbase_m, track_width_m, wheel_radius_m

auto state = model.bodyVelocityToSteering(2.0, 0.2);       // linear m/s, angular rad/s
// state.steering_angle_rad, state.turning_radius_m
// state.front_right_wheel_rad_s, state.front_left_wheel_rad_s
// state.rear_right_wheel_rad_s, state.rear_left_wheel_rad_s

auto body = model.steeringToBodyVelocity(2.0, 0.15);       // velocity m/s, steering_angle rad
// body.linear_velocity_m_s, body.angular_velocity_rad_s

double radius = model.turningRadius(0.15);                  // steering_angle rad -> meters
double angle  = model.steeringAngleFromRadius(10.0);        // radius m -> radians
```

### ArticulatedModel

```cpp
polymath::kinematics::ArticulatedModel model(
    1.8, 1.5,   // articulation_to_front_axle_m, articulation_to_rear_axle_m
    2.0, 2.0,   // front_track_width_m, rear_track_width_m
    0.6, 0.6);  // front_wheel_radius_m, rear_wheel_radius_m

auto state = model.bodyVelocityToVehicleState(1.5, 0.1);   // linear m/s, angular rad/s
// state.articulation_angle_rad
// state.front_right_wheel_speed_rad_s, state.front_left_wheel_speed_rad_s
// state.rear_right_wheel_speed_rad_s, state.rear_left_wheel_speed_rad_s
// state.front_axle_turning_radius_m, state.rear_axle_turning_radius_m

auto axles = model.articulationToAxleVelocities(1.5, 0.3); // linear m/s, articulation_angle rad
// axles.front_axle_turning_velocity_rad_s, axles.rear_axle_turning_velocity_rad_s
```

## Python usage

```python
from polymath_kinematics import DifferentialDriveModel, BicycleModel, ArticulatedModel
```

### DifferentialDriveModel

```python
model = DifferentialDriveModel(wheel_radius_m=0.15, track_width_m=0.5)

wheels = model.body_velocity_to_wheel_velocities(linear_vel=1.0, angular_vel=0.3)
print(wheels.left_wheel_velocity_rad_s, wheels.right_wheel_velocity_rad_s)

body = model.wheel_velocities_to_body_velocity(left_wheel_vel=6.0, right_wheel_vel=7.0)
print(body.linear_velocity_m_s, body.angular_velocity_rad_s)
```

### BicycleModel

```python
model = BicycleModel(wheelbase_m=2.7, track_width_m=1.6, wheel_radius_m=0.35)

state = model.body_velocity_to_steering(linear_velocity=2.0, angular_velocity=0.2)
print(state.steering_angle_rad, state.turning_radius_m)

body = model.steering_to_body_velocity(velocity=2.0, steering_angle=0.15)
print(body.linear_velocity_m_s, body.angular_velocity_rad_s)

radius = model.turning_radius(steering_angle=0.15)
angle  = model.steering_angle_from_radius(radius=10.0)
```

### ArticulatedModel

```python
model = ArticulatedModel(
    articulation_to_front_axle_m=1.8,
    articulation_to_rear_axle_m=1.5,
    front_track_width_m=2.0,
    rear_track_width_m=2.0,
    front_wheel_radius_m=0.6,
    rear_wheel_radius_m=0.6,
)

state = model.body_velocity_to_vehicle_state(
    linear_velocity_m_s=1.5, angular_velocity_rad_s=0.1
)
print(state.articulation_angle_rad, state.front_axle_turning_radius_m)

axles = model.articulation_to_axle_velocities(
    linear_velocity_m_s=1.5, articulation_angle_rad=0.3
)
print(axles.front_axle_turning_velocity_rad_s, axles.rear_axle_turning_velocity_rad_s)
```

## Models reference

### DifferentialDriveModel

| Constructor param | Description |
|---|---|
| `wheel_radius_m` | Wheel radius in meters |
| `track_width_m` | Distance between wheel centers |

| Method (C++ / Python) | Parameters | Returns |
|---|---|---|
| `bodyVelocityToWheelVelocities` / `body_velocity_to_wheel_velocities` | linear vel (m/s), angular vel (rad/s) | `DifferentialDriveWheelVelocities` |
| `wheelVelocitiesToBodyVelocity` / `wheel_velocities_to_body_velocity` | left wheel vel (rad/s), right wheel vel (rad/s) | `DifferentialDriveBodyVelocity` |

### BicycleModel

| Constructor param | Description |
|---|---|
| `wheelbase_m` | Front-to-rear axle distance |
| `track_width_m` | Distance between left and right wheels |
| `wheel_radius_m` | Wheel radius in meters |

| Method (C++ / Python) | Parameters | Returns |
|---|---|---|
| `bodyVelocityToSteering` / `body_velocity_to_steering` | linear vel (m/s), angular vel (rad/s) | `BicycleSteeringState` |
| `steeringToBodyVelocity` / `steering_to_body_velocity` | velocity (m/s), steering angle (rad) | `BicycleBodyVelocity` |
| `turningRadius` / `turning_radius` | steering angle (rad) | `double` / `float` |
| `steeringAngleFromRadius` / `steering_angle_from_radius` | radius (m) | `double` / `float` |

### ArticulatedModel

| Constructor param | Description |
|---|---|
| `articulation_to_front_axle_m` | Joint-to-front-axle distance |
| `articulation_to_rear_axle_m` | Joint-to-rear-axle distance |
| `front_track_width_m` | Front axle wheel spacing |
| `rear_track_width_m` | Rear axle wheel spacing |
| `front_wheel_radius_m` | Front wheel radius |
| `rear_wheel_radius_m` | Rear wheel radius |

| Method (C++ / Python) | Parameters | Returns |
|---|---|---|
| `bodyVelocityToVehicleState` / `body_velocity_to_vehicle_state` | linear vel (m/s), angular vel (rad/s) | `ArticulatedVehicleState` |
| `articulationToAxleVelocities` / `articulation_to_axle_velocities` | linear vel (m/s), articulation angle (rad) | `ArticulatedAxleVelocities` |
