# polymath_kinematics

Kinematic models and forward-projection helpers for differential-drive, bicycle (Ackermann), and articulated vehicles. C++ library with Python bindings via pybind11, packaged as a single Python wheel built by [scikit-build-core].

Includes an interactive **Kinematic Explorer** (Streamlit) for visualising trajectory lattices, kinematic relationships, and vehicle footprints.

[scikit-build-core]: https://scikit-build-core.readthedocs.io/

## Installation

**Prerequisites:** Python 3.10+, a C++17 compiler, CMake 3.15+, and [uv](https://github.com/astral-sh/uv).

```bash
cd src/polymath_kinematics

# C++ bindings only
uv pip install -e .

# Bindings + Kinematic Explorer (streamlit, matplotlib, pandas)
uv pip install -e ".[explorer]"

# Bindings + Explorer + pytest
uv pip install -e ".[dev]"
```

The first install builds the pybind11 extension via CMake; subsequent installs are incremental.

### ROS2 / colcon

The package also builds inside a colcon workspace (it has a top-level `CMakeLists.txt`, no `package.xml` — colcon picks it up via its CMake detector):

```bash
# From the workspace root
colcon build --merge-install --packages-select polymath_kinematics
```

To also build the C++ tests:

```bash
colcon build --merge-install --packages-select polymath_kinematics --cmake-args -DBUILD_TESTING=ON
```

## Running the Kinematic Explorer

After `uv pip install -e ".[explorer]"`:

```bash
# Console script (recommended)
kinematic-explorer

# Or invoke streamlit directly
uv run python -m streamlit run polymath_kinematics/kinematic_explorer_app.py
```

The app opens at `http://localhost:8501` and exposes:

- Model selection (Differential Drive / Bicycle / Articulated)
- Geometry sliders (wheelbase, track width, wheel radii, etc.)
- Trajectory lattice visualisation across steering / articulation angles
- Kinematic analysis plots (angle → angular velocity, turning radius vs angle)
- Vehicle-footprint overlays along trajectories
- CSV / JSON / PNG / SVG / PDF export

## Running tests

```bash
# Python (bindings + explorer)
uv run pytest

# C++ (via colcon)
colcon build --merge-install --packages-select polymath_kinematics --cmake-args -DBUILD_TESTING=ON
cd ../../build/polymath_kinematics && ctest --output-on-failure
```

## Derivations

Math and equations behind each model:

- [Differential drive](derivations/differential_drive.md) — forward/inverse kinematics for two independently driven wheels
- [Bicycle model (Ackermann)](derivations/bicycle_model.md) — steering-angle kinematics with four-wheel ICR geometry
- [Articulated model](derivations/articulated_model.md) — pivot-joint kinematics for front/rear section vehicles

## Python usage

```python
from polymath_kinematics import (
    DifferentialDriveModel,
    BicycleModel,
    ArticulatedModel,
    BicycleProjector,
    ArticulatedProjector,
    Pose2D,
)
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

# Steady articulation (gamma-dot = 0)
state = model.body_velocity_to_vehicle_state(
    linear_velocity_m_s=1.5, angular_velocity_rad_s=0.1
)
print(state.articulation_angle_rad, state.front_axle_turning_radius_m)

# With explicit articulation rate (gamma-dot, rad/s)
state_with_rate = model.body_velocity_to_vehicle_state(
    linear_velocity_m_s=1.5,
    angular_velocity_rad_s=0.1,
    articulation_turning_velocity_rad_s=0.2,
)

axles = model.articulation_to_axle_velocities(
    linear_velocity_m_s=1.5,
    articulation_angle_rad=0.3,
    articulation_turning_velocity_rad_s=0.2,  # optional, defaults to 0
)
print(axles.front_axle_turning_velocity_rad_s, axles.rear_axle_turning_velocity_rad_s)
```

### Projectors — forward simulation with steering-rate limits

`BicycleProjector` and `ArticulatedProjector` wrap their respective models and integrate pose forward in time under realistic actuator constraints: the steering / articulation angle ramps toward a target at a bounded rate, clamped to `[min, max]`.

```python
projector = BicycleProjector(
    model=BicycleModel(wheelbase_m=2.7, track_width_m=1.6, wheel_radius_m=0.35),
    min_steering_angle_rad=-0.6,
    max_steering_angle_rad=0.6,
)

# One-step advance
result = projector.step(
    dt_s=0.1,
    current_pose=Pose2D(x=0.0, y=0.0, theta=0.0),
    current_steering_angle_rad=0.0,
    target_steering_angle_rad=0.3,
    steering_rate_rad_s=0.5,
    linear_velocity_m_s=1.0,
)
print(result.pose.x, result.steering_angle_rad)

# Full trajectory
trajectory = projector.project(
    horizon_s=5.0,
    dt_s=0.05,
    initial_pose=Pose2D(),
    initial_steering_angle_rad=0.0,
    target_steering_angle_rad=0.4,
    steering_rate_rad_s=0.3,
    linear_velocity_m_s=1.5,
)
# trajectory[0] is the initial state; trajectory[-1] is the end-of-horizon state.
```

`ArticulatedProjector` has the same shape — the angle is the articulation angle γ and the rate is γ̇:

```python
projector = ArticulatedProjector(
    model=ArticulatedModel(1.66, 1.44, 2.0, 2.0, 0.723, 0.723),
    min_articulation_angle_rad=-0.785,
    max_articulation_angle_rad=0.785,
)
trajectory = projector.project(
    horizon_s=5.0, dt_s=0.05,
    initial_pose=Pose2D(),
    initial_articulation_angle_rad=0.0,
    target_articulation_angle_rad=0.5,
    articulation_rate_rad_s=0.2,
    linear_velocity_m_s=1.0,
)
```

## C++ usage

The C++ headers and library are installed under the package's CMake export when built via colcon:

```cmake
find_package(polymath_kinematics REQUIRED)
target_link_libraries(my_target PRIVATE polymath_kinematics::polymath_kinematics)
```

```cpp
#include <polymath_kinematics/differential_drive_model.hpp>
#include <polymath_kinematics/bicycle_model.hpp>
#include <polymath_kinematics/articulated_model.hpp>
#include <polymath_kinematics/bicycle_projector.hpp>
#include <polymath_kinematics/articulated_projector.hpp>
#include <polymath_kinematics/pose2d.hpp>
```

### DifferentialDriveModel

```cpp
polymath::kinematics::DifferentialDriveModel model(0.15, 0.5);  // wheel_radius_m, track_width_m

auto wheels = model.bodyVelocityToWheelVelocities(1.0, 0.3);   // linear m/s, angular rad/s
auto body   = model.wheelVelocitiesToBodyVelocity(6.0, 7.0);   // left rad/s, right rad/s
```

### BicycleModel

```cpp
polymath::kinematics::BicycleModel model(2.7, 1.6, 0.35);  // wheelbase, track, wheel_radius (m)

auto state = model.bodyVelocityToSteering(2.0, 0.2);       // linear m/s, angular rad/s
auto body  = model.steeringToBodyVelocity(2.0, 0.15);      // velocity m/s, steering rad

double radius = model.turningRadius(0.15);                  // steering rad → meters
double angle  = model.steeringAngleFromRadius(10.0);        // radius m → radians
```

### ArticulatedModel

```cpp
polymath::kinematics::ArticulatedModel model(
    1.8, 1.5,   // articulation_to_front_axle_m, articulation_to_rear_axle_m
    2.0, 2.0,   // front_track_width_m, rear_track_width_m
    0.6, 0.6);  // front_wheel_radius_m, rear_wheel_radius_m

// 2-arg form: assumes a zero articulation turning velocity
auto state = model.bodyVelocityToVehicleState(1.5, 0.1);

// 3-arg form: feeds the actual gamma-dot into the kinematics
auto state_with_rate = model.bodyVelocityToVehicleState(1.5, 0.1, 0.2);

auto axles = model.articulationToAxleVelocities(1.5, 0.3);            // gamma-dot = 0
auto axles_with_rate = model.articulationToAxleVelocities(1.5, 0.3, 0.2);
```

### Projectors

```cpp
polymath::kinematics::BicycleProjector projector(
    polymath::kinematics::BicycleModel(2.7, 1.6, 0.35),
    -0.6, 0.6);  // min/max steering angle (rad)

polymath::kinematics::Pose2D pose{0.0, 0.0, 0.0};

auto step = projector.step(
    /*dt_s=*/0.1, pose,
    /*current_steering=*/0.0,
    /*target_steering=*/0.3,
    /*steering_rate=*/0.5,
    /*linear_velocity=*/1.0);

auto trajectory = projector.project(
    /*horizon_s=*/5.0, /*dt_s=*/0.05, pose,
    /*initial_steering=*/0.0,
    /*target_steering=*/0.4,
    /*steering_rate=*/0.3,
    /*linear_velocity=*/1.5);
// trajectory.front() is the initial state, trajectory.back() is the end of the horizon.
```

`ArticulatedProjector` is shaped identically — the angle is the articulation angle (γ) and the rate is γ̇.

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
| `bodyVelocityToVehicleState` / `body_velocity_to_vehicle_state` | linear vel (m/s), angular vel (rad/s), [articulation rate (rad/s) = 0] | `ArticulatedVehicleState` |
| `articulationToAxleVelocities` / `articulation_to_axle_velocities` | linear vel (m/s), articulation angle (rad), [articulation rate (rad/s) = 0] | `ArticulatedAxleVelocities` |

### Projectors

| Class | Constructor | Notable methods |
|---|---|---|
| `BicycleProjector` | `(BicycleModel, min_steering, max_steering)` | `step(dt, pose, current, target, rate, v)`, `project(horizon, dt, pose, initial, target, rate, v)` |
| `ArticulatedProjector` | `(ArticulatedModel, min_articulation, max_articulation)` | `step(dt, pose, current, target, rate, v)`, `project(horizon, dt, pose, initial, target, rate, v)` |

Both projectors clamp the target angle to `[min, max]` before ramping, then advance the current angle toward the clamped target at the given rate (never overshooting), and integrate pose with Euler.
