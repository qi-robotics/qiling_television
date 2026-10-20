#pragma once

#include <algorithm>
#include <cmath>

#include "qiling_kinematics/ik_types.hpp"

namespace qiling_kinematics
{

// A held command must already point into the safe region before Grip may
// re-engage while feedback is near a URDF limit. This never admits feedback
// outside the hard URDF range.
inline bool canResumeNearLimit(
  const ArmVector & measured, const ArmVector & reference,
  const ArmVector & q_min, const ArmVector & q_max,
  double margin, double max_tracking_error)
{
  if (!measured.allFinite() || !reference.allFinite() ||
    !q_min.allFinite() || !q_max.allFinite() ||
    !std::isfinite(margin) || margin <= 0.0 ||
    !std::isfinite(max_tracking_error) || max_tracking_error < 0.0)
  {
    return false;
  }

  for (int i = 0; i < kSingleArmDof; ++i) {
    if (q_min[i] >= q_max[i] || 2.0 * margin >= q_max[i] - q_min[i] ||
      measured[i] < q_min[i] || measured[i] > q_max[i] ||
      reference[i] < q_min[i] || reference[i] > q_max[i] ||
      std::abs(reference[i] - measured[i]) > max_tracking_error)
    {
      return false;
    }
    if (measured[i] < q_min[i] + margin && reference[i] < q_min[i] + margin) {
      return false;
    }
    if (measured[i] > q_max[i] - margin && reference[i] > q_max[i] - margin) {
      return false;
    }
  }
  return true;
}

// The QP still uses the URDF hard limits. At a near-limit feedback position,
// also suppress any solved velocity that would move farther toward that limit.
inline ArmVector suppressOutwardVelocityNearLimit(
  const ArmVector & measured, const ArmVector & qdot,
  const ArmVector & q_min, const ArmVector & q_max, double margin)
{
  ArmVector guarded = qdot;
  for (int i = 0; i < kSingleArmDof; ++i) {
    if (measured[i] <= q_min[i] + margin) {
      guarded[i] = std::max(guarded[i], 0.0);
    }
    if (measured[i] >= q_max[i] - margin) {
      guarded[i] = std::min(guarded[i], 0.0);
    }
  }
  return guarded;
}

}  // namespace qiling_kinematics
