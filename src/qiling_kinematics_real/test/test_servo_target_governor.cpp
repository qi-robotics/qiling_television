#include <gtest/gtest.h>

#include "qiling_kinematics/servo_target_governor.hpp"

namespace qiling_kinematics
{
namespace
{
TEST(ServoTargetGovernor, DropsUnexecutableControllerMotion)
{
  ServoTargetGovernor governor;
  ServoTargetGovernorConfig config;
  const Eigen::Isometry3d robot = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d raw = robot;
  EXPECT_NEAR(governor.update(raw, robot, 0.0, 0.02, config).translation().x(), 0.0, 1e-12);

  raw.translation().x() = 1.0;
  const auto bounded = governor.update(raw, robot, 0.0, 0.02, config);
  EXPECT_NEAR(bounded.translation().x(), 0.004, 1e-12);
  const auto stopped_input = governor.update(raw, robot, 0.0, 0.02, config);
  EXPECT_NEAR(stopped_input.translation().x(), bounded.translation().x(), 1e-12);

  raw.translation().x() = 1.1;
  const auto tracking_stopped = governor.update(raw, robot, 0.30, 0.02, config);
  EXPECT_NEAR(tracking_stopped.translation().x(), bounded.translation().x(), 1e-12);
  // The displacement discarded during a tracking stop cannot spring back.
  EXPECT_NEAR(
    governor.update(raw, robot, 0.0, 0.02, config).translation().x(),
    bounded.translation().x(), 1e-12);
}

TEST(ServoTargetGovernor, KeepsAdmittedTargetNearRobot)
{
  ServoTargetGovernor governor;
  ServoTargetGovernorConfig config;
  const Eigen::Isometry3d robot = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d raw = robot;
  governor.update(raw, robot, 0.0, 0.02, config);
  for (int i = 0; i < 100; ++i) {
    raw.translation().x() += 0.01;
    const auto target = governor.update(raw, robot, 0.0, 0.02, config);
    EXPECT_LE(target.translation().norm(), config.max_position_lead_m + 1e-12);
  }
}
}  // namespace
}  // namespace qiling_kinematics
