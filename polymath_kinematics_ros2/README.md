# polymath_kinematics_ros2

ROS 2 layer over [polymath_kinematics](../polymath_kinematics/).

**Placeholder.** `KinematicsNode` is a `LifecycleNode` whose transition callbacks are no-ops. It
declares no parameters, topics, or services — it exists so the build target, component
registration, and link against the models are already in place.

```bash
ros2 run polymath_kinematics_ros2 kinematics_node
```

Registration uses upstream `rclcpp_components_register_node`, and the test plain Catch2, rather
than polymath_core's `rclcpp_lifecycle_components_register_node` and `polymath_test`. Both of
those live in polymath_core and are unavailable when this repository builds standalone in CI.
