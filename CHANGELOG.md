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

- `Pose2D`, `Point2D`, and `Footprint` (a 4-corner, counter-clockwise, unclosed world-frame
  polygon) in a shared `pose2d.hpp`, along with `normalizeAngle`.
- Each projector optionally computes a per-sample body footprint from its own dimensions; the
  kinematic models stay dimension-free. A body width of 0 or less emits an empty footprint
  rather than throwing. The articulated projector emits separate front and rear polygons.

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
- Geometry sliders (including body overhangs), single-projected-trajectory ramp controls, and
  CSV / JSON / PNG / SVG / PDF download.

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
