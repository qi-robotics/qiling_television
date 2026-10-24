#include "qiling_kinematics/servo_target_governor.hpp"

#include <algorithm>
#include <cmath>

namespace qiling_kinematics
{
namespace
{
Eigen::Vector3d boundedVector(const Eigen::Vector3d & vector, double limit)
{
  const double length = vector.norm();
  return length > limit && length > 1.0e-12 ? vector * (limit / length) : vector;
}

Eigen::Matrix3d boundedRotation(const Eigen::Matrix3d & rotation, double limit)
{
  const Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle()) || angle_axis.angle() < 1.0e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(
    std::min(angle_axis.angle(), limit), angle_axis.axis()).toRotationMatrix();
}
}  // namespace

void ServoTargetGovernor::reset()
{
  initialized_ = false;
}

Eigen::Isometry3d ServoTargetGovernor::update(
  const Eigen::Isometry3d & raw_target,
  const Eigen::Isometry3d & measured_wrist,
  double tracking_error_rad, double dt,
  const ServoTargetGovernorConfig & config)
{
  if (!raw_target.matrix().allFinite() || !measured_wrist.matrix().allFinite() ||
    !std::isfinite(tracking_error_rad) || !std::isfinite(dt) || dt <= 0.0 ||
    config.max_linear_velocity_mps <= 0.0 || config.max_angular_velocity_rps <= 0.0 ||
    config.max_position_lead_m <= 0.0 || config.max_orientation_lead_rad <= 0.0 ||
    config.tracking_stop_rad <= config.tracking_slowdown_start_rad)
  {
    reset();
    return measured_wrist;
  }

  if (!initialized_) {
    previous_raw_ = raw_target;
    admitted_ = measured_wrist;
    initialized_ = true;
    return admitted_;
  }

  const double scale = std::clamp(
    (config.tracking_stop_rad - tracking_error_rad) /
    (config.tracking_stop_rad - config.tracking_slowdown_start_rad), 0.0, 1.0);
  const Eigen::Vector3d raw_delta =
    raw_target.translation() - previous_raw_.translation();
  const Eigen::Matrix3d raw_rotation_delta =
    raw_target.rotation() * previous_raw_.rotation().transpose();
  previous_raw_ = raw_target;  // Drop excess input; never accumulate a catch-up target.

  Eigen::Isometry3d candidate = admitted_;
  candidate.translation() += boundedVector(
    raw_delta, scale * config.max_linear_velocity_mps * dt);
  candidate.linear() = boundedRotation(
    raw_rotation_delta, scale * config.max_angular_velocity_rps * dt) *
    admitted_.rotation();

  const Eigen::Vector3d lead_limited_position =
    measured_wrist.translation() + boundedVector(
    candidate.translation() - measured_wrist.translation(), config.max_position_lead_m);
  const Eigen::Matrix3d lead_limited_rotation = boundedRotation(
    candidate.rotation() * measured_wrist.rotation().transpose(),
    config.max_orientation_lead_rad) * measured_wrist.rotation();
  candidate.translation() = admitted_.translation() + boundedVector(
    lead_limited_position - admitted_.translation(),
    scale * config.max_linear_velocity_mps * dt);
  candidate.linear() = boundedRotation(
    lead_limited_rotation * admitted_.rotation().transpose(),
    scale * config.max_angular_velocity_rps * dt) * admitted_.rotation();
  admitted_ = candidate;
  return admitted_;
}

}  // namespace qiling_kinematics
