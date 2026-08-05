# MATLAB kinematics prototype

Reference implementation of the quadruped leg kinematics, written in MATLAB
before being ported to C++/Eigen as a ROS 2 package. Kept in the repo as the
origin of that port and as the source of its regression tests.

## Layout
- `params_quad.m` — single source of truth for the model (SI units)
- `startup.m` — path setup, run first
- `src/1_kinematics/` — FK, IK and the analytic Jacobian, plus Robotics System
  Toolbox (`*_rst`) counterparts used as an independent oracle
- `viz/` — plotting helpers that rebuild the chain from scratch, so they check
  the kinematics rather than re-plotting them
- `verify_phase01.m`, `run_all.m` — asserting acceptance tests

## Conventions
Hip frame: origin on the hip-abduction (HAA) axis, aligned with the body frame
at zero angles — x forward, y left, z up. Joints in order `q1` HAA (about x),
`q2` HFE (about y), `q3` KFE (about y):

    Rx(q1) · [0; s·l_abad; 0] · Ry(q2) · [0; 0; -l_thigh] · Ry(q3) · [0; 0; -l_calf]

with `s = +1` for left legs (FL, RL) and `-1` for right (FR, RR). `leg_fk`
implements this as a product of exponentials; `leg_ik` inverts it in closed
form, fixing the knee branch to `q3 < 0` and resolving the remaining abduction
ambiguity against a seed pose.

## Run

    matlab -batch "startup; run_all"

## Tests
`verify_phase01` asserts, for all four legs: the zero-pose and bent-knee anchors,
200 randomized `FK -> IK -> FK` round-trips within 1e-9, 25 analytic-vs-central-
difference Jacobians within 1e-6, branch fidelity against a seeded pose, joint
limits, graceful clamping of unreachable targets, and agreement with the
Robotics System Toolbox model. These assertions are what get ported to gtest in
the C++ package.
