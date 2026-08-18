# quadruped-stack

Leg kinematics, vendor-SDK integration and timing instrumentation for a
quadruped robot, developed against a pinned, reproducible ROS 2 (Humble)
container environment.

The repository is built package by package in the open. Everything below that is
marked as working has an accompanying test or a recorded measurement; nothing is
claimed before it runs.

## Status

|Component|State|
|-|-|
|`matlab/` — closed-form leg FK / IK / analytic Jacobian|working, asserted by `verify\_phase01`|
|Docker + ROS 2 Humble development environment|working — digest-pinned image, `colcon build` verified|
|`leg\_kinematics` — C++/Eigen port as a ROS 2 package|working — FK/IK header, node on `/joint\_states` → `/foot\_positions` + TF|
|gtest regression suite + CI|planned|
|Unitree SDK adapter against `unitree\_mujoco`|planned|
|LiDAR-inertial odometry on recorded bags|planned|
|IMU/LiDAR time-synchronization and latency monitor|planned|

## What works today

`matlab/` holds the reference implementation of the leg kinematics for a
Mini-Cheetah-class 3-DOF leg: hip abduction, hip flexion, knee flexion.

* **Forward kinematics** as a product of exponentials, so the model is a list of
screw axes and a home pose rather than a hand-expanded trigonometric formula.
* **Inverse kinematics** in closed form: abduction from the `(y, z)` projection,
then the planar two-link problem by the law of cosines. The knee branch is
fixed to `q3 < 0`, and the remaining abduction ambiguity — both branches are
reachable on this robot — is resolved against a seed pose so the solution
stays continuous through a control loop.
* **Analytic Jacobian** derived by hand and checked against a central finite
difference of the forward kinematics.
* **Unreachable targets clamp rather than throw**, because a controller running
at kilohertz has to degrade, not error.

Every one of those properties is asserted, for all four legs, in
`matlab/verify\_phase01.m`, alongside a parity check against an independently
constructed Robotics System Toolbox model. See `matlab/README.md` for the frame
conventions and how to run it.

That code is the specification for the C++ port; the MATLAB assertions become
the gtest suite.

## Layout

```
matlab/   MATLAB kinematics prototype and its acceptance tests
src/      ROS 2 workspace source (packages land here)
docs/     design notes
```

## License

MIT — see `LICENSE`.

