# polymath_kinematics

Kinematic models and forward-projection helpers for differential-drive, bicycle (Ackermann), and articulated vehicles. C++ library with Python bindings via pybind11, packaged as a single Python wheel built by [scikit-build-core].

Includes an interactive **Kinematic Explorer** (Streamlit) for visualising trajectory lattices, kinematic relationships, and vehicle footprints.

[scikit-build-core]: https://scikit-build-core.readthedocs.io/

## Installation

**Prerequisites:** Python 3.10+, a C++17 compiler, CMake 3.15+, and [uv](https://github.com/astral-sh/uv) for the wheel path.

The package supports three build paths. They share one `CMakeLists.txt`; `find_package(ament_cmake QUIET)` picks between the ROS2 and standalone branches at configure time.

### 1. Python wheel (uv / pip)

For Python-only consumers and the Streamlit explorer. Builds a static `polymath_kinematics` library and links it into the `polymath_kinematics_cpp` pybind11 module; ships the result as a single wheel.

```bash
cd src/polymath_kinematics

# Bindings only
uv pip install -e .

# Bindings + Kinematic Explorer (streamlit, matplotlib, pandas)
uv pip install -e ".[explorer]"

# Bindings + Explorer + pytest
uv pip install -e ".[dev]"

# Or, without activating a venv first:
uv run python -c "import polymath_kinematics; print(polymath_kinematics.BicycleModel(2.5, 1.5, 0.3))"
```

The wheel build sets `-DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON` via `pyproject.toml`, so a host-installed `/opt/ros/humble` does not pull the build down the ROS2 branch.

### 2. ROS2 / colcon

For consumers inside a ROS2 workspace. Produces a shared `libpolymath_kinematics.so` exported through ament (`polymath_kinematics::polymath_kinematics`) plus the pybind11 module installed alongside the Python package.

```bash
# From the colcon workspace root
colcon build --packages-select polymath_kinematics

# Build + run C++ Catch2 tests and Python pytest suites
colcon build --packages-select polymath_kinematics --cmake-args -DBUILD_TESTING=ON
colcon test  --packages-select polymath_kinematics
colcon test-result --verbose
```

### 3. Standalone CMake

For working on the C++ code or running the Catch2 tests outside of ROS2 and outside of the wheel build (e.g. local IDE / ctest workflows).

```bash
cd src/polymath_kinematics
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

If `/opt/ros/humble` (or any other ament-providing install) is on the host, force the standalone path with:

```bash
cmake -S . -B build -DBUILD_TESTING=ON -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
```

## Running the Kinematic Explorer

After `uv pip install -e ".[explorer]"`:

```bash
# Console script (recommended). Extra arguments are forwarded to streamlit.
kinematic-explorer
kinematic-explorer --server.port=8600

# Or invoke streamlit directly
uv run python -m streamlit run polymath_kinematics/kinematic_explorer_app.py
```

The app opens at `http://localhost:8501` and exposes:

- Model selection (Differential Drive / Bicycle / Articulated)
- Geometry sliders (wheelbase, track width, wheel radii, body overhangs) and a front/rear axle reference selector
- Trajectory lattice visualisation across steering / articulation angles
- Kinematic analysis plots (angle → angular velocity, turning radius vs angle)
- Vehicle-footprint overlays along trajectories, drawn from the projector-computed polygons
- Single projected trajectory with initial → target ramp controls
- CSV / JSON / PNG / SVG / PDF download

Every trajectory the explorer draws comes from the C++ projectors, so it exercises the same
forward-simulation code as production. Integration is Euler at the configured time step.

The console script binds to `localhost` by default; pass `--server.address=0.0.0.0` to serve a
demo over the network.

## Running tests

```bash
# Python only (from the wheel install)
uv run pytest

# C++ Catch2 + Python pytest under ROS2
colcon build --packages-select polymath_kinematics --cmake-args -DBUILD_TESTING=ON
colcon test  --packages-select polymath_kinematics
colcon test-result --verbose

# C++ Catch2 standalone (no ROS2)
cmake -S . -B build -DBUILD_TESTING=ON -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
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
    DifferentialDriveProjector,
    BicycleProjector,
    ArticulatedProjector,
    AxleReference,
    Pose2D,
    Point2D,
    rectangle_footprint,
    transform_footprint,
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

### Projectors — forward simulation with actuator limits

Each projector wraps its model and integrates pose forward in time under realistic actuator
constraints. For `BicycleProjector` and `ArticulatedProjector` the steering / articulation angle
ramps toward a target at a bounded rate, clamped to `[min, max]`. `DifferentialDriveProjector`
has no steered joint, so it instead ramps the body command `(v, ω)` under separate linear and
angular acceleration limits.

Every projector optionally computes a per-sample vehicle footprint. Footprints are **arbitrary
polygons** given in the body frame, and they live on the projector rather than the model — the
kinematic models stay dimension-free. A polygon is counter-clockwise and not closed; an empty
polygon (the default) means "unset", so the footprint comes back empty and projection proceeds
normally. `rectangle_footprint(front_m, rear_m, width_m)` builds the boxy common case, and
`transform_footprint(polygon, pose)` maps a body-frame polygon into another frame if you need to
transform one yourself.

For the bicycle and articulated models, poses and the body-frame footprint are both measured from
an axle you choose with `AxleReference.FRONT` or `AxleReference.REAR`. A differential drive has one
axle, so there is nothing to select.

| Projector | Pose reference | Footprint |
|---|---|---|
| `DifferentialDriveProjector` | Body centre | One polygon in the body-centre frame |
| `BicycleProjector` | Selected axle (`AxleReference`) | One polygon in the selected axle's frame |
| `ArticulatedProjector` | Selected axle (`AxleReference`); θ is that axle's body heading | One polygon per body, **each in its own axle's frame** — the only anchoring that stays rigid as the joint articulates |

`ArticulatedProjector` also reports `joint_pose` on every sample, so the articulation joint
(`base_link` for a ROS articulated vehicle) is available regardless of which axle you reference.
See the [derivations](#derivations) for the per-model geometry and integration details.

```python
projector = BicycleProjector(
    model=BicycleModel(wheelbase_m=2.7, track_width_m=1.6, wheel_radius_m=0.35),
    min_steering_angle_rad=-0.6,
    max_steering_angle_rad=0.6,
    axle_reference=AxleReference.REAR,
    # Measured from the rear axle: front bumper 0.9 m past the 2.7 m front axle, 0.8 m of tail.
    footprint=rectangle_footprint(2.7 + 0.9, 0.8, 1.8),
)

# Or describe the same body from the front axle — the polygon moves with the reference:
front_referenced = BicycleProjector(
    model=BicycleModel(wheelbase_m=2.7, track_width_m=1.6, wheel_radius_m=0.35),
    min_steering_angle_rad=-0.6,
    max_steering_angle_rad=0.6,
    axle_reference=AxleReference.FRONT,
    footprint=rectangle_footprint(0.9, 2.7 + 0.8, 1.8),
)

# Any polygon works, not just rectangles — a tapered nose, for instance:
tapered = [Point2D(-0.8, -0.9), Point2D(3.2, -0.9), Point2D(3.6, 0.0), Point2D(3.2, 0.9), Point2D(-0.8, 0.9)]

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
print([(p.x, p.y) for p in result.footprint])  # world-frame body polygon; empty if no footprint was given

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
    axle_reference=AxleReference.REAR,
    # Front body about the FRONT axle: 1.0 m of bucket ahead, back to the joint 1.66 m behind.
    front_footprint=rectangle_footprint(1.0, 1.66, 2.0),
    # Rear body about the REAR axle: forward to the joint 1.44 m ahead, 0.8 m of counterweight.
    rear_footprint=rectangle_footprint(1.44, 0.8, 2.0),
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

`DifferentialDriveProjector` ramps the body command instead of an angle:

```python
projector = DifferentialDriveProjector(
    model=DifferentialDriveModel(wheel_radius_m=0.15, track_width_m=0.5),
    min_linear_velocity_m_s=-2.0,
    max_linear_velocity_m_s=2.0,
    min_angular_velocity_rad_s=-1.5,
    max_angular_velocity_rad_s=1.5,
)
trajectory = projector.project(
    horizon_s=5.0, dt_s=0.05,
    initial_pose=Pose2D(),
    initial_linear_velocity_m_s=0.0,
    initial_angular_velocity_rad_s=0.0,
    target_linear_velocity_m_s=1.0,
    target_angular_velocity_rad_s=0.5,
    linear_acceleration_m_s2=1.0,
    angular_acceleration_rad_s2=1.0,
)
# Setting initial == target makes the ramp a no-op, giving a constant-command trajectory.
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
#include <polymath_kinematics/differential_drive_projector.hpp>
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
    -0.6, 0.6,                                            // min/max steering angle (rad)
    polymath::kinematics::AxleReference::REAR,             // poses + footprint about the rear axle
    polymath::kinematics::rectangleFootprint(3.6, 0.8, 1.8));  // front_m, rear_m, width_m

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

`ArticulatedProjector` is shaped identically — the angle is the articulation angle (γ) and the rate is γ̇. `DifferentialDriveProjector` takes a body command and acceleration limits instead:

```cpp
polymath::kinematics::DifferentialDriveProjector diff_projector(
    polymath::kinematics::DifferentialDriveModel(0.15, 0.5),
    -2.0, 2.0,    // min/max linear velocity (m/s)
    -1.5, 1.5);   // min/max angular velocity (rad/s)

auto diff_trajectory = diff_projector.project(
    /*horizon_s=*/5.0, /*dt_s=*/0.05, pose,
    /*initial_v=*/0.0, /*initial_omega=*/0.0,
    /*target_v=*/1.0, /*target_omega=*/0.5,
    /*linear_accel=*/1.0, /*angular_accel=*/1.0);
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
| `bodyVelocityToVehicleState` / `body_velocity_to_vehicle_state` | linear vel (m/s), angular vel (rad/s), [articulation rate (rad/s) = 0] | `ArticulatedVehicleState` |
| `articulationToAxleVelocities` / `articulation_to_axle_velocities` | linear vel (m/s), articulation angle (rad), [articulation rate (rad/s) = 0] | `ArticulatedAxleVelocities` |

### Projectors

| Class | Constructor | Notable methods |
|---|---|---|
| `BicycleProjector` | `(BicycleModel, min_steering, max_steering, [axle_reference, footprint])` | `step(dt, pose, current, target, rate, v)`, `project(horizon, dt, pose, initial, target, rate, v)` |
| `ArticulatedProjector` | `(ArticulatedModel, min_articulation, max_articulation, [axle_reference, front_footprint, rear_footprint])` | `step(dt, pose, current, target, rate, v)`, `project(horizon, dt, pose, initial, target, rate, v)` |
| `DifferentialDriveProjector` | `(DifferentialDriveModel, min_v, max_v, min_omega, max_omega, [footprint])` | `step(dt, pose, current_v, current_omega, target_v, target_omega, accel, angular_accel)`, `project(horizon, dt, pose, initial_v, initial_omega, target_v, target_omega, accel, angular_accel)` |

All three clamp the target to its `[min, max]` bounds before ramping, then advance toward the clamped target at the given rate or acceleration (never overshooting), and integrate pose with Euler. Each step advances position using the heading `θ` at the *start* of the step, while the angular rate `ω` comes from the *post-ramp* steering/articulation angle — the model headers phrase this as integrating with the "post-ramp angle", referring to that rate, not the heading used for position. `project()` returns `ceil(horizon / dt) + 1` samples, with the initial state as element 0; it returns an empty sequence for degenerate inputs (`dt_s <= 0` or `horizon_s < 0`).

`BicycleProjector` and `ArticulatedProjector` ramp an angle; `DifferentialDriveProjector` ramps the body command `(v, ω)` under separate linear and angular acceleration limits, since a differential drive has no steered joint.
