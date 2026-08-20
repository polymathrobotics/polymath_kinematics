# polymath_kinematics_python_tools

The **Kinematic Explorer** — a Streamlit app for visualising trajectory lattices, kinematic
relationships, and vehicle footprints, built on the [polymath_kinematics](../polymath_kinematics/)
bindings. Trajectories come from the C++ projectors, integrated with Euler at the configured time
step.

Pure Python, managed with uv. Not a ROS 2 package.

## Install and run

Needs Python 3.10+, [uv](https://github.com/astral-sh/uv), and — because `polymath_kinematics`
builds from source — a C++17 compiler and CMake 3.15+.

```bash
uv sync                      # from the repository root
uv run kinematic-explorer    # extra arguments are forwarded to streamlit
```

The app opens at `http://localhost:8501`. It binds to `localhost`; pass
`--server.address=0.0.0.0` to serve over the network.

## Tests

```bash
uv run pytest polymath_kinematics_python_tools/test/test_explorer.py
```
