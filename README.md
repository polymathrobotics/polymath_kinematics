# polymath_kinematics

Kinematic models for differential-drive, bicycle (Ackermann), and articulated vehicles. Three
self-contained projects:

| Project | What it is | Built with |
| --- | --- | --- |
| [polymath_kinematics](polymath_kinematics/) | C++17 models and projectors, with pybind11 bindings | colcon, or a wheel |
| [polymath_kinematics_python_tools](polymath_kinematics_python_tools/) | Kinematic Explorer — Streamlit app, simulation, plotting, export | uv |
| [polymath_kinematics_ros2](polymath_kinematics_ros2/) | ROS 2 layer — **placeholder**, no interfaces yet | colcon |

## Quickstart

### Python (uv)

The repository root is a uv workspace, so one sync covers both Python projects.

```bash
uv sync
uv run kinematic-explorer
```

### ROS 2 (colcon)

```bash
colcon build --packages-select polymath_kinematics polymath_kinematics_ros2
colcon test  --packages-select polymath_kinematics polymath_kinematics_ros2
```

### Standalone C++

```bash
cmake -S polymath_kinematics -B build -DBUILD_TESTING=ON \
      -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON
cmake --build build -j && ctest --test-dir build --output-on-failure
```
