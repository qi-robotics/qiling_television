#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

#include "qiling_kinematics/ik_types.hpp"

namespace qiling_kinematics
{

enum class GestureKind {Wave, Shake, Baoquan, Home, Zero};
enum class GesturePhase {Entry, GripWait, Motion, ReturnHome, Complete};

const char * gestureName(GestureKind kind);

struct GestureFrame
{
  std::array<ArmVector, 2> position;
  std::array<ArmVector, 2> velocity;
  std::array<ArmVector, 2> acceleration;
  GesturePhase phase{GesturePhase::Entry};
};

// Pure joint-space trajectory: no ROS, DDS, hardware or command publishing.
class GesturePlan
{
public:
  static GesturePlan load(
    const std::filesystem::path & config_path, GestureKind kind,
    const std::array<ArmVector, 2> & start,
    const std::array<ArmVector, 2> & home,
    const std::array<ArmVector, 2> & lower,
    const std::array<ArmVector, 2> & upper,
    const ArmVector & speed_limit,
    const ArmVector & acceleration_limit);

  GestureFrame sample(double elapsed_sec) const;
  double totalDuration() const;
  GestureKind kind() const {return kind_;}
  const std::array<ArmVector, 2> & endPose() const {return home_;}
  std::array<double, 3> segmentDurations() const {return durations_;}

private:
  struct Term {double amplitude; double phase;};
  using JointTerms = std::array<std::vector<Term>, kSingleArmDof>;
  using SideTerms = std::array<JointTerms, 2>;

  GestureFrame segmentSample(GesturePhase phase, double u, double duration) const;
  double retimedDuration(GesturePhase phase, double minimum) const;
  void validate() const;

  GestureKind kind_{GestureKind::Wave};
  std::array<ArmVector, 2> start_{};
  std::array<ArmVector, 2> target_{};
  std::array<ArmVector, 2> home_{};
  std::array<ArmVector, 2> lower_{};
  std::array<ArmVector, 2> upper_{};
  ArmVector speed_limit_{ArmVector::Zero()};
  ArmVector acceleration_limit_{ArmVector::Zero()};
  SideTerms terms_{};
  std::array<double, 3> durations_{}; // entry, motion, return
  double grip_wait_sec_{0.0};
  double cycles_{0.0};
  double margin_{0.0};
  double safety_factor_{1.0};
};

// Offline collision checker. Returns an empty string
// on success or the first offending geometry pair / model error on failure.
std::string checkGestureCollisions(
  const GesturePlan & plan,
  const std::filesystem::path & urdf_path,
  const std::filesystem::path & package_parent);

std::string prewarmArmCollisionGeometry(
  const std::filesystem::path & urdf_path,
  const std::filesystem::path & package_parent);

}  // namespace qiling_kinematics
