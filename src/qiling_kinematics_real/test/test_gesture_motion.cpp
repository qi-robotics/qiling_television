#include <array>
#include <filesystem>

#include <gtest/gtest.h>

#include "qiling_kinematics/dual_arm_kinematics.hpp"
#include "qiling_kinematics/gesture_motion.hpp"

namespace qiling_kinematics
{
namespace
{
using Arms = std::array<ArmVector, 2>;

const std::filesystem::path kPackage =
  std::filesystem::path(__FILE__).parent_path().parent_path();
const std::filesystem::path kRoot = kPackage.parent_path().parent_path();
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

GesturePlan loadPlan(GestureKind kind, const Arms & start)
{
  DualArmKinematics model(kUrdf);
  const Arms lower = {
    model.lowerPositionLimits(ArmSide::Left),
    model.lowerPositionLimits(ArmSide::Right)};
  const Arms upper = {
    model.upperPositionLimits(ArmSide::Left),
    model.upperPositionLimits(ArmSide::Right)};
  ArmVector speed;
  speed << 1.0, 1.0, 1.0, 1.2, 1.2, 1.2, 1.2;
  ArmVector acceleration;
  acceleration << 3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0;
  return GesturePlan::load(
    kPackage / "config/gesture_actions.yaml", kind, start, home(),
    lower, upper, speed, acceleration);
}

TEST(GestureMotion, ThreeActionsReturnHomeWithoutBoundaryJumps)
{
  for (GestureKind kind : {GestureKind::Wave, GestureKind::Shake, GestureKind::Baoquan}) {
    const auto plan = loadPlan(kind, home());
    const auto durations = plan.segmentDurations();
    EXPECT_NEAR(
      plan.totalDuration(), durations[0] + durations[1] + durations[2] +
      (kind == GestureKind::Shake ? 1.0 : 0.0), 1e-9);
    const auto initial = plan.sample(0.0);
    const auto final = plan.sample(plan.totalDuration());
    for (int side = 0; side < 2; ++side) {
      EXPECT_LT((initial.position[side] - home()[side]).norm(), 1e-10);
      EXPECT_LT((final.position[side] - home()[side]).norm(), 1e-10);
      EXPECT_LT(initial.velocity[side].norm(), 1e-10);
      EXPECT_LT(final.velocity[side].norm(), 1e-10);
      EXPECT_LT(plan.sample(durations[0]).velocity[side].norm(), 1e-10);
      EXPECT_LT(
        plan.sample(durations[0] + durations[1] +
          (kind == GestureKind::Shake ? 1.0 : 0.0)).velocity[side].norm(), 1e-10);
    }
  }
}

TEST(GestureMotion, RejectsStartingOutsideSafetyMargin)
{
  Arms unsafe = home();
  unsafe[1][0] = -3.0;
  EXPECT_THROW(loadPlan(GestureKind::Wave, unsafe), std::runtime_error);
}

TEST(GestureMotion, HomeMovesInwardFromNearRightElbowLimit)
{
  Arms start = home();
  start[1][3] = -1.91304;  // Held command from the NX limit-margin rejection log.
  const auto plan = loadPlan(GestureKind::Home, start);
  const auto durations = plan.segmentDurations();
  EXPECT_GE(durations[0], 3.0);
  EXPECT_NEAR(plan.sample(0.0).position[1][3], -1.91304, 1e-9);
  EXPECT_NEAR(plan.sample(plan.totalDuration()).position[1][3], home()[1][3], 1e-9);
  for (double time = 0.0; time <= durations[0]; time += 0.02) {
    const double elbow = plan.sample(time).position[1][3];
    EXPECT_GE(elbow, -1.91304 - 1e-9);
    EXPECT_LE(elbow, home()[1][3] + 1e-9);
  }
}

TEST(GestureMotion, HomeRejectsStartOutsideUrdfHardLimit)
{
  Arms start = home();
  start[1][3] = -2.10;
  EXPECT_THROW(loadPlan(GestureKind::Home, start), std::runtime_error);
}

TEST(GestureMotion, WaveHasCollisionFreeModelPreflightAtConfiguredHome)
{
  const auto plan = loadPlan(GestureKind::Wave, home());
  EXPECT_TRUE(checkGestureCollisions(plan, kUrdf, kRoot / "src").empty());
}

}  // namespace
}  // namespace qiling_kinematics
