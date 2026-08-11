#pragma once

// Closed-form kinematics of one 3-DOF quadruped leg (HAA, HFE, KFE).
// Direct port of matlab/src/1_kinematics/leg_fk.m and leg_ik.m, which are the
// reference implementation and the source of this file's regression tests.
//
// Hip frame: origin on the HAA axis, aligned with the body frame at q = 0
// (x forward, y left, z up). The chain is
//
//   Rx(q1) * [0; s*l_abad; 0] * Ry(q2) * [0; 0; -l_thigh] * Ry(q3) * [0; 0; -l_calf]
//
// with s = +1 for left legs (FL, RL) and -1 for right (FR, RR).
//
// Deliberately free of ROS types: the ROS node, the tests and the vendor SDK
// adapter all use this header unchanged.

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace leg_kinematics
{

/// Geometry and joint limits of one leg. Mirrors matlab/params_quad.m.
struct LegParams
{
  double l_abad = 0.0;     ///< HAA axis -> thigh mount, lateral offset (m)
  double l_thigh = 0.0;    ///< HFE -> knee (m)
  double l_calf = 0.0;     ///< knee -> foot (m)
  double abad_sign = 1.0;  ///< +1 for left legs (FL, RL), -1 for right (FR, RR)

  Eigen::Vector3d hip = Eigen::Vector3d::Zero();  ///< hip mount in the body frame (m)

  Eigen::Vector3d q_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d q_max = Eigen::Vector3d::Zero();
  Eigen::Vector3d q_nominal = Eigen::Vector3d::Zero();
};

/// Which abduction branch a solution sits on: foot below the hip, or above it.
enum class Branch { kDown, kUp };

/// What the IK returns. The C++ shape of leg_ik.m's [q, info].
struct IkResult
{
  Eigen::Vector3d q = Eigen::Vector3d::Zero();
  bool reachable = true;   ///< false if the target is outside the workspace (q is then clamped)
  bool in_limits = true;   ///< whether q lies inside [q_min, q_max]
  Branch branch = Branch::kDown;
};

/// True if every joint lies inside its limits, with a tolerance for round-off.
inline bool inLimits(const Eigen::Vector3d & q, const LegParams & p)
{
  constexpr double kTol = 1e-12;
  return (q.array() >= p.q_min.array() - kTol).all() &&
         (q.array() <= p.q_max.array() + kTol).all();
}

/// Forward kinematics: joint angles -> foot position in the HIP frame.
///
/// Anchor: q = 0 gives [0; s*l_abad; -(l_thigh + l_calf)] — the leg straight down.
inline Eigen::Vector3d footPositionFromJoints(const Eigen::Vector3d & q, const LegParams & p)
{
  const Eigen::Matrix3d R1 = Eigen::AngleAxisd(q(0), Eigen::Vector3d::UnitX()).toRotationMatrix();
  const Eigen::Matrix3d R2 = Eigen::AngleAxisd(q(1), Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d R3 = Eigen::AngleAxisd(q(2), Eigen::Vector3d::UnitY()).toRotationMatrix();

  const Eigen::Vector3d abad(0.0, p.abad_sign * p.l_abad, 0.0);
  const Eigen::Vector3d thigh(0.0, 0.0, -p.l_thigh);
  const Eigen::Vector3d calf(0.0, 0.0, -p.l_calf);

  return R1 * (abad + R2 * (thigh + R3 * calf));
}

/// Inverse kinematics: foot position in the HIP frame -> joint angles.
///
/// Abduction from the (y, z) projection, then the planar two-link problem in the
/// (x, R) plane by the law of cosines. The knee branch is pinned to q3 < 0.
///
/// The remaining abduction ambiguity cannot be resolved from p_foot alone: both
/// the foot-below-hip and foot-above-hip solutions reproduce it exactly, and both
/// are reachable on this robot. Both are therefore computed, and the one nearest
/// q_seed is returned, with out-of-limits solutions pushed behind in-limits ones.
/// In a control loop, seed with the previous solution so the branch stays
/// continuous.
///
/// Unreachable targets are clamped, never thrown: a kilohertz controller has to
/// degrade rather than error. Two ways to miss — inside the cylinder of radius
/// l_abad about the HAA axis, or outside the two-link annulus — and both set
/// reachable = false.
inline IkResult jointsFromFootPosition(
  const Eigen::Vector3d & p_foot, const LegParams & p, const Eigen::Vector3d & q_seed)
{
  const double s = p.abad_sign;
  const double la = p.l_abad;
  const double lt = p.l_thigh;
  const double lc = p.l_calf;
  const double x = p_foot.x();
  const double y = p_foot.y();
  const double z = p_foot.z();

  // (y, z) plane: R is the leg seen from the front, |z'| of the leg-plane coordinate.
  const double d2 = y * y + z * z;
  const double R = std::sqrt(std::max(d2 - la * la, 0.0));  // max(): clamp inside the cylinder

  // (x, R) plane: law of cosines. The clamp is what stops acos returning NaN.
  const double L2 = x * x + R * R;
  const double c3 = (L2 - lt * lt - lc * lc) / (2.0 * lt * lc);
  const double q3 = -std::acos(std::clamp(c3, -1.0, 1.0));  // negative: knee bends backward

  const double psi = std::atan2(lc * std::sin(q3), lt + lc * std::cos(q3));

  // Solution for a given signed leg-plane coordinate zp = z'.
  const auto branchAt = [&](double zp) {
      return Eigen::Vector3d(
        std::atan2(s * la * z - zp * y, s * la * y + zp * z),
        std::atan2(-x, -zp) - psi,
        q3);
    };

  const Eigen::Vector3d q_down = branchAt(-R);  // foot below the hip
  const Eigen::Vector3d q_up = branchAt(+R);    // foot above it

  // Nearest to the seed, with anything out of limits pushed behind everything
  // inside them. Ties fall to 'down', the stance branch.
  constexpr double kOutOfLimits = 1e3;  // larger than any in-box joint distance
  const double cost_down = (q_down - q_seed).norm() + (inLimits(q_down, p) ? 0.0 : kOutOfLimits);
  const double cost_up = (q_up - q_seed).norm() + (inLimits(q_up, p) ? 0.0 : kOutOfLimits);

  IkResult out;
  if (cost_up < cost_down) {
    out.q = q_up;
    out.branch = Branch::kUp;
  } else {
    out.q = q_down;
    out.branch = Branch::kDown;
  }

  // Tolerated so a target sitting a micron inside a boundary — where FK output
  // lands when the leg plane degenerates — still counts as reachable.
  constexpr double kTol = 1e-12;
  out.reachable = (d2 >= la * la - kTol) && (std::abs(c3) <= 1.0 + kTol);
  out.in_limits = inLimits(out.q, p);
  return out;
}

}  // namespace leg_kinematics
