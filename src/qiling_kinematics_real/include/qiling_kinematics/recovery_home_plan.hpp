#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

#include "qiling_kinematics/ik_types.hpp"

namespace qiling_kinematics
{

struct RecoveryHomeFrame
{
  std::array<ArmVector, 2> position{};
  std::array<ArmVector, 2> velocity{};
};

// A measured-state reconciliation followed by a bounded path to home.
// The path is planned without ROS or hardware access and can be preflighted.
class RecoveryHomePlan
{
public:
  static RecoveryHomePlan create(
    const std::array<ArmVector, 2> & last_command,
    const std::array<ArmVector, 2> & measured,
    const std::array<ArmVector, 2> & home,
    const std::array<ArmVector, 2> & lower,
    const std::array<ArmVector, 2> & upper,
    const ArmVector & max_velocity,
    const ArmVector & max_acceleration,
    bool via_zero);

  RecoveryHomeFrame sample(double elapsed_sec) const;
  double totalDuration() const {return total_duration_;}
  double reconcileDuration() const {return durations_.empty() ? 0.0 : durations_.front();}
  bool viaZero() const {return via_zero_;}

private:
  std::vector<std::array<ArmVector, 2>> waypoints_;
  std::vector<double> durations_;
  double total_duration_{0.0};
  bool via_zero_{false};
};

// Returns an empty string when every sampled arm configuration is clear.
std::string checkRecoveryHomeCollisions(
  const RecoveryHomePlan & plan,
  const std::filesystem::path & urdf_path,
  const std::filesystem::path & package_parent);

}  // namespace qiling_kinematics
