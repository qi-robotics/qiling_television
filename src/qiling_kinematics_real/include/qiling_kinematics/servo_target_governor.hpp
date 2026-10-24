#pragma once

#include <Eigen/Geometry>

namespace qiling_kinematics
{

// Converts controller displacement into a bounded, disposable servo request.
// Motion discarded while the robot is saturated is never queued for later.
struct ServoTargetGovernorConfig
{
  double max_linear_velocity_mps{0.20};
  double max_angular_velocity_rps{0.80};
  double max_position_lead_m{0.08};
  double max_orientation_lead_rad{0.30};
  double tracking_slowdown_start_rad{0.10};
  double tracking_stop_rad{0.25};
};

class ServoTargetGovernor
{
public:
  void reset();
  Eigen::Isometry3d update(
    const Eigen::Isometry3d & raw_target,
    const Eigen::Isometry3d & measured_wrist,
    double tracking_error_rad, double dt,
    const ServoTargetGovernorConfig & config);
  bool initialized() const {return initialized_;}

private:
  Eigen::Isometry3d previous_raw_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d admitted_{Eigen::Isometry3d::Identity()};
  bool initialized_{false};
};

}  // namespace qiling_kinematics
