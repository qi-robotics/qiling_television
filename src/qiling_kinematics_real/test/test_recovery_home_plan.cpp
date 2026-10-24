#include <array>
#include <filesystem>

#include <gtest/gtest.h>

#include "qiling_kinematics/dual_arm_kinematics.hpp"
#include "qiling_kinematics/recovery_home_plan.hpp"

namespace qiling_kinematics
{
namespace
{
using Arms = std::array<ArmVector, 2>;
const std::filesystem::path kRoot =
  std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path();
const std::filesystem::path kUrdf =
  kRoot / "src/qi_robot_description/urdf/s4_dual_arm.urdf";

Arms home()
{
  Arms result;
  result[0] << 0.0104905777, 0.6509880424, -0.1741435826,
    -1.7084382772, 0.0738155171, -0.0963225737, -0.0936522484;
  result[1] << -0.0749599487, -0.6475547552, 0.1073853672,
    -1.2964446545, -0.0009536889, -0.4251545072, 0.4945830405;
  return result;
}

TEST(RecoveryHomePlan, StartsAtLastCommandAndEndsAtHomeWithinJointRates)
{
  DualArmKinematics model(kUrdf);
  const Arms lower = {model.lowerPositionLimits(ArmSide::Left),
    model.lowerPositionLimits(ArmSide::Right)};
  const Arms upper = {model.upperPositionLimits(ArmSide::Left),
    model.upperPositionLimits(ArmSide::Right)};
  Arms commanded = home();
  Arms measured = home();
  commanded[1] << -1.37009, 0.0362572, 0.293457, 0.57755,
    0.0649014, 0.590813, -0.643186;
  measured[1] << -1.58141, 0.0139239, 0.286297, 0.22755,
    0.0375753, 0.534257, -0.602541;
  ArmVector velocity;
  velocity << 1.0, 1.0, 1.0, 1.2, 1.2, 1.2, 1.2;
  ArmVector acceleration;
  acceleration << 3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0;
  const auto plan = RecoveryHomePlan::create(
    commanded, measured, home(), lower, upper, velocity, acceleration, false);
  EXPECT_TRUE(plan.sample(0.0).position[1].isApprox(commanded[1]));
  EXPECT_TRUE(plan.sample(plan.reconcileDuration()).position[1].isApprox(measured[1]));
  EXPECT_TRUE(plan.sample(plan.totalDuration()).position[1].isApprox(home()[1]));
  for (double t = 0.0; t <= plan.totalDuration(); t += 0.02) {
    const auto frame = plan.sample(t);
    for (int side = 0; side < 2; ++side) {
      for (int joint = 0; joint < kSingleArmDof; ++joint) {
        EXPECT_LE(std::abs(frame.velocity[side][joint]), velocity[joint] + 1e-9);
        EXPECT_GE(frame.position[side][joint], lower[side][joint] - 1e-9);
        EXPECT_LE(frame.position[side][joint], upper[side][joint] + 1e-9);
      }
    }
  }
  EXPECT_TRUE(checkRecoveryHomeCollisions(plan, kUrdf, kRoot / "src").empty());
}

TEST(RecoveryHomePlan, HistoricalLargeTrackingGapHasAValidatedRoute)
{
  DualArmKinematics model(kUrdf);
  const Arms lower = {model.lowerPositionLimits(ArmSide::Left),
    model.lowerPositionLimits(ArmSide::Right)};
  const Arms upper = {model.upperPositionLimits(ArmSide::Left),
    model.upperPositionLimits(ArmSide::Right)};
  Arms commanded = home();
  Arms measured = home();
  commanded[1] << -1.0708, 0.60797, 1.96768, 0.315095,
    1.05574, 0.636612, 0.77722;
  measured[1] << -1.2911, -0.185969, 1.95029, -0.632296,
    1.05459, 0.63077, 0.778019;
  ArmVector velocity;
  velocity << 1.0, 1.0, 1.0, 1.2, 1.2, 1.2, 1.2;
  ArmVector acceleration;
  acceleration << 3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0;
  const auto direct = RecoveryHomePlan::create(
    commanded, measured, home(), lower, upper, velocity, acceleration, false);
  const std::string direct_error = checkRecoveryHomeCollisions(direct, kUrdf, kRoot / "src");
  if (!direct_error.empty()) {
    const auto zero = RecoveryHomePlan::create(
      commanded, measured, home(), lower, upper, velocity, acceleration, true);
    EXPECT_TRUE(checkRecoveryHomeCollisions(zero, kUrdf, kRoot / "src").empty()) <<
      "direct route: " << direct_error;
  }
}
}  // namespace
}  // namespace qiling_kinematics
