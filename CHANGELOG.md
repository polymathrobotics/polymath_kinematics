# Changelog

## v0.2.0

### Forward projection

- `BicycleProjector` — ramps the steering angle toward a target at a bounded rate, clamped to
  `[min, max]`, and integrates pose with Euler. Pose reference is the rear axle.
- `ArticulatedProjector` — same shape for the articulation angle γ and rate γ̇. Pose reference
  (`base_link`) is the articulation joint; motion is integrated at the rear axle and converted
  back to the joint each step.
- `DifferentialDriveProjector` — ramps the body command (v, ω) toward a target under separate
  linear and angular acceleration limits. Pose reference is the body centre.
- All three expose `step()` for one time step and `project()` for a full horizon, returning a
  timestamped state sequence whose element 0 is the initial state.

### Footprints

- `Pose2D`, `Point2D`, and `Footprint` (a counter-clockwise, unclosed polygon of any vertex count)
  in a shared `pose2d.hpp`, along with `normalizeAngle`, `transformFootprint`,
  `offsetAlongHeading`, and `rectangleFootprint` for the boxy common case.
- Each projector optionally takes an **arbitrary body-frame polygon** and emits it transformed into
  the world frame per sample; the kinematic models stay dimension-free. An empty polygon means
  "unset" and yields an empty footprint rather than throwing. The articulated projector takes one
  polygon per body.

### Axle reference

- `AxleReference::FRONT` / `REAR` selects the axle that `BicycleProjector` and
  `ArticulatedProjector` measure poses and footprints from. Motion is still integrated where each
  model is naturally defined (the rear axle); the reference conversion is applied on input and
  output, so no integration accuracy is traded for the choice.
- For the articulated model, `theta` follows the referenced axle's body — $\theta_r$ for REAR,
  $\theta_f = \theta_r + \gamma$ for FRONT — and each body's polygon is anchored at **its own**
  axle, the only anchoring that stays rigid as the joint articulates.
- `ArticulatedProjectedState` gained `joint_pose`, so the articulation joint (`base_link`) is
  reported on every sample whichever axle is referenced.
- A differential drive has one axle, so `DifferentialDriveProjector` takes no reference; its
  polygon is measured from the body centre.

### Kinematics

- `ArticulatedModel` now accepts an explicit articulation turning velocity (γ̇) on
  `bodyVelocityToVehicleState` and `articulationToAxleVelocities`. The previous two-argument
  forms remain as overloads that assume steady articulation (γ̇ = 0), so existing callers are
  unaffected. γ̇ was previously a hardcoded zero constant.

### Packaging

- Wheel build via scikit-build-core, alongside the existing ROS2/ament path. One
  `CMakeLists.txt` serves both; `find_package(ament_cmake QUIET)` selects the branch. The wheel
  links the C++ library statically so it ships a single `.so`.
- Python bindings for all three projectors, their projected-state structs, `Pose2D`, and
  `Point2D`.

### Kinematic Explorer

- New Streamlit app (`kinematic-explorer`) for visualising trajectory lattices, kinematic
  relationships, and vehicle footprints across all three models. Every trajectory it draws
  comes from the C++ projectors, so the explorer and production code share one
  forward-simulation path.
- Geometry sliders (including body overhangs), a front/rear axle reference selector,
  single-projected-trajectory ramp controls, and CSV / JSON / PNG / SVG / PDF download.

### Removed

- `Twist2D`, `normalize_angle`, and `transform_pose` from the Python package — leftovers from
  when `Pose2D` was a Python shim, now superseded by the bound C++ `Pose2D` and
  `normalizeAngle`.

## v0.1.0

Initial release.

- `DifferentialDriveModel` — forward/inverse kinematics for two independently driven wheels
- `BicycleModel` — Ackermann steering kinematics with four-wheel ICR geometry
- `ArticulatedModel` — pivot-joint kinematics for front/rear section vehicles
- Python bindings for all three models via pybind11
