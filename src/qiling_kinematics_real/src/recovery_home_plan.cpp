#include "qiling_kinematics/recovery_home_plan.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace qiling_kinematics
{
namespace
{
double segmentDuration(
  const std::array<ArmVector, 2> & from,
  const std::array<ArmVector, 2> & to,
  const ArmVector & velocity,
  const ArmVector & acceleration,
  double minimum)
{
  double duration = minimum;
  for (int side = 0; side < 2; ++side) {
    for (int joint = 0; joint < kSingleArmDof; ++joint) {
      const double distance = std::abs(to[side][joint] - from[side][joint]);
      duration = std::max(duration, 1.875 * distance / velocity[joint]);
      duration = std::max(duration, std::sqrt(6.0 * distance / acceleration[joint]));
    }
  }
  return std::ceil(1.10 * duration / 0.02) * 0.02;
}
}  // namespace

RecoveryHomePlan RecoveryHomePlan::create(
  const std::array<ArmVector, 2> & last_command,
  const std::array<ArmVector, 2> & measured,
  const std::array<ArmVector, 2> & home,
  const std::array<ArmVector, 2> & lower,
  const std::array<ArmVector, 2> & upper,
  const ArmVector & max_velocity,
  const ArmVector & max_acceleration,
  bool via_zero)
{
  if (!max_velocity.allFinite() || !max_acceleration.allFinite() ||
    (max_velocity.array() <= 0.0).any() || (max_acceleration.array() <= 0.0).any())
  {
    throw std::invalid_argument("invalid recovery joint rate limits");
  }
  RecoveryHomePlan plan;
  plan.via_zero_ = via_zero;
  plan.waypoints_.push_back(last_command);
  std::array<ArmVector, 2> reconciled = measured;
  for (int side = 0; side < 2; ++side) {
    if (!last_command[side].allFinite() || !measured[side].allFinite() ||
      !home[side].allFinite() || !lower[side].allFinite() || !upper[side].allFinite())
    {
      throw std::invalid_argument("non-finite recovery configuration");
    }
    for (int joint = 0; joint < kSingleArmDof; ++joint) {
      if (lower[side][joint] >= upper[side][joint] ||
        last_command[side][joint] < lower[side][joint] ||
        last_command[side][joint] > upper[side][joint] ||
        home[side][joint] < lower[side][joint] ||
        home[side][joint] > upper[side][joint] ||
        (via_zero && (0.0 < lower[side][joint] || 0.0 > upper[side][joint])))
      {
        throw std::invalid_argument("recovery waypoint outside URDF joint limits");
      }
      // Feedback slightly beyond a hard limit must be commanded inward,
      // never echoed as an out-of-range position command.
      reconciled[side][joint] = std::clamp(
        measured[side][joint], lower[side][joint], upper[side][joint]);
    }
  }
  plan.waypoints_.push_back(reconciled);
  if (via_zero) {
    plan.waypoints_.push_back({ArmVector::Zero(), ArmVector::Zero()});
  }
  plan.waypoints_.push_back(home);
  for (std::size_t i = 1; i < plan.waypoints_.size(); ++i) {
    const double duration = segmentDuration(
      plan.waypoints_[i - 1], plan.waypoints_[i], max_velocity,
      max_acceleration, i == 1 ? 0.75 : 2.0);
    plan.durations_.push_back(duration);
    plan.total_duration_ += duration;
  }
  return plan;
}

RecoveryHomeFrame RecoveryHomePlan::sample(double elapsed_sec) const
{
  if (!std::isfinite(elapsed_sec) || waypoints_.size() < 2) {
    throw std::invalid_argument("invalid recovery sample");
  }
  RecoveryHomeFrame frame;
  double remaining = std::max(0.0, elapsed_sec);
  for (std::size_t segment = 0; segment < durations_.size(); ++segment) {
    const double duration = durations_[segment];
    if (remaining <= duration || segment + 1 == durations_.size()) {
      const double u = std::clamp(remaining / duration, 0.0, 1.0);
      const double blend = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
      const double blend_rate =
        (30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u) / duration;
      for (int side = 0; side < 2; ++side) {
        const ArmVector delta =
          waypoints_[segment + 1][side] - waypoints_[segment][side];
        frame.position[side] = waypoints_[segment][side] + blend * delta;
        frame.velocity[side] = blend_rate * delta;
      }
      return frame;
    }
    remaining -= duration;
  }
  throw std::logic_error("recovery plan has no segments");
}

}  // namespace qiling_kinematics
