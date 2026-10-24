#include <gtest/gtest.h>

#include "qiling_kinematics/arm_control_state.hpp"
#include "qiling_kinematics/near_limit_guard.hpp"

namespace
{

using qiling_kinematics::ArmControlState;
using qiling_kinematics::ArmRunState;
using qiling_kinematics::ArmVector;
using qiling_kinematics::canResumeNearLimit;
using qiling_kinematics::suppressOutwardVelocityNearLimit;

TEST(NearLimitGuard, GuardedRegripMatchesObservedLeftElbowAndKeepsInwardMotion)
{
  ArmVector q_min = ArmVector::Constant(-2.0);
  ArmVector q_max = ArmVector::Constant(2.0);
  q_min[3] = -2.007;
  q_max[3] = 1.57;
  ArmVector measured = ArmVector::Zero();
  ArmVector reference = ArmVector::Zero();
  measured[3] = -1.96479;
  reference[3] = -1.93325;

  ASSERT_TRUE(canResumeNearLimit(measured, reference, q_min, q_max, 0.05, 0.35));
  ArmControlState left("left");
  left.initialize(reference);
  left.updateGripRequest(true, measured);
  ASSERT_TRUE(left.active());
  left.updateGripRequest(false, measured);
  ASSERT_EQ(left.state(), ArmRunState::Hold);
  ASSERT_FALSE(left.releaseRequired());
  EXPECT_TRUE(left.reference().isApprox(reference));
  ASSERT_TRUE(canResumeNearLimit(
      measured, left.reference(), q_min, q_max, 0.05, 0.35));
  left.updateGripRequest(true, measured);
  EXPECT_EQ(left.state(), ArmRunState::Active);
  EXPECT_TRUE(left.reference().isApprox(reference));

  ArmVector qdot = ArmVector::Constant(0.1);
  qdot[3] = -0.2;
  const ArmVector guarded = suppressOutwardVelocityNearLimit(
    measured, qdot, q_min, q_max, 0.05);
  EXPECT_DOUBLE_EQ(guarded[3], 0.0);
  EXPECT_DOUBLE_EQ(guarded[0], 0.1);

  qdot[3] = 0.2;
  EXPECT_DOUBLE_EQ(suppressOutwardVelocityNearLimit(
      measured, qdot, q_min, q_max, 0.05)[3], 0.2);
}

TEST(NearLimitGuard, RejectsUnsafeReferenceAndHardLimitViolation)
{
  const ArmVector q_min = ArmVector::Constant(-2.0);
  const ArmVector q_max = ArmVector::Constant(2.0);
  ArmVector measured = ArmVector::Zero();
  ArmVector reference = ArmVector::Zero();
  measured[3] = -1.97;
  reference[3] = -1.97;
  EXPECT_FALSE(canResumeNearLimit(measured, reference, q_min, q_max, 0.05, 0.35));

  reference[3] = -1.90;
  measured[3] = -2.001;
  EXPECT_FALSE(canResumeNearLimit(measured, reference, q_min, q_max, 0.05, 0.35));
}

TEST(NearLimitGuard, AppliesSymmetricallyAtUpperLimit)
{
  const ArmVector q_min = ArmVector::Constant(-2.0);
  const ArmVector q_max = ArmVector::Constant(2.0);
  ArmVector measured = ArmVector::Zero();
  ArmVector reference = ArmVector::Zero();
  measured[2] = 1.97;
  reference[2] = 1.90;
  ASSERT_TRUE(canResumeNearLimit(measured, reference, q_min, q_max, 0.05, 0.35));
  ArmVector qdot = ArmVector::Constant(0.1);
  qdot[2] = 0.2;
  EXPECT_DOUBLE_EQ(suppressOutwardVelocityNearLimit(
      measured, qdot, q_min, q_max, 0.05)[2], 0.0);
}

TEST(ArmControlState, HoldNeverIntegrates)
{
  ArmControlState state("left");
  const ArmVector measured = ArmVector::Constant(0.2);
  state.initialize(measured);

  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));

  const ArmVector qdot = ArmVector::Ones();
  const ArmVector q_min = ArmVector::Constant(-2.0);
  const ArmVector q_max = ArmVector::Constant(2.0);
  EXPECT_FALSE(state.integrateReference(qdot, 0.02, measured, q_min, q_max, 0.35));
  EXPECT_TRUE(state.reference().isApprox(measured));
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));
}

TEST(ArmControlState, TrackingExemptionDoesNotRemoveJointPositionLimits)
{
  ArmControlState state("left");
  const ArmVector measured = ArmVector::Zero();
  state.initialize(measured);
  state.updateGripRequest(true, measured);

  const ArmVector q_min = ArmVector::Constant(-1.0);
  const ArmVector q_max = ArmVector::Constant(1.0);
  ArmVector qdot = ArmVector::Zero();
  qdot[6] = 1.0;

  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(state.integrateReference(qdot, 0.1, measured, q_min, q_max, 0.2, 6));
  }
  EXPECT_NEAR(state.reference()[6], 0.5, 1.0e-12);

  qdot[6] = 10.0;
  ASSERT_TRUE(state.integrateReference(qdot, 0.1, measured, q_min, q_max, 0.2, 6));
  EXPECT_NEAR(state.reference()[6], 1.0, 1.0e-12);
}

TEST(ArmControlState, GripReleasePreservesReferenceAndResetsVelocity)
{
  ArmControlState state("left");
  const ArmVector measured = ArmVector::Zero();
  state.initialize(measured);
  state.updateGripRequest(true, measured);
  ASSERT_TRUE(state.active());

  const ArmVector qdot = ArmVector::Constant(0.5);
  const ArmVector q_min = ArmVector::Constant(-2.0);
  const ArmVector q_max = ArmVector::Constant(2.0);
  ASSERT_TRUE(state.integrateReference(qdot, 0.02, measured, q_min, q_max, 0.35));
  EXPECT_FALSE(state.reference().isZero(0.0));

  const ArmVector released_measured = ArmVector::Constant(0.1);
  state.updateGripRequest(false, released_measured);
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  const ArmVector held_reference = ArmVector::Constant(0.01);
  EXPECT_TRUE(state.reference().isApprox(held_reference));
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));

  state.updateGripRequest(true, released_measured);
  EXPECT_EQ(state.state(), ArmRunState::Active);
  EXPECT_TRUE(state.reference().isApprox(held_reference));
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));
}

TEST(ArmControlState, FaultReleaseWaitsForExplicitRecovery)
{
  ArmControlState state("left");
  const ArmVector measured = ArmVector::Zero();
  state.initialize(measured);
  state.updateGripRequest(true, measured);
  ASSERT_TRUE(state.active());

  const ArmVector fault_measured = ArmVector::Constant(0.2);
  state.enterFault(fault_measured, "reference integration failed");
  ASSERT_EQ(state.state(), ArmRunState::Fault);

  const ArmVector released_measured = ArmVector::Constant(0.3);
  state.updateGripRequest(false, released_measured);
  EXPECT_EQ(state.state(), ArmRunState::Fault);
  EXPECT_TRUE(state.releaseRequired());
  EXPECT_TRUE(state.reference().isApprox(measured));
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));

  state.completeRecovery(released_measured);
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_TRUE(state.reference().isApprox(released_measured));
}

TEST(ArmControlState, HoldKeepsLatchedReference)
{
  ArmControlState state("left");
  const ArmVector home = ArmVector::Constant(0.2);
  state.initialize(home);

  state.enterHold(home, false, "home reached");
  const ArmVector drooped = ArmVector::Constant(0.35);
  state.enterHold(drooped, false, "Grip released");

  EXPECT_TRUE(state.reference().isApprox(home));
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));
}

TEST(ArmControlState, SynchronizeReferencePreservesRunStateAndReleaseLatch)
{
  ArmControlState state("left");
  const ArmVector initial = ArmVector::Constant(0.2);
  state.initialize(initial);
  state.updateGripRequest(true, initial);
  ASSERT_TRUE(state.active());

  const ArmVector home = ArmVector::Constant(-0.4);
  state.synchronizeReference(home, "home handoff");
  EXPECT_EQ(state.state(), ArmRunState::Active);
  EXPECT_FALSE(state.releaseRequired());
  EXPECT_TRUE(state.reference().isApprox(home));
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));

  state.enterHold(initial, true, "recording boundary");
  state.synchronizeReference(home, "home handoff while release required");
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_TRUE(state.releaseRequired());
  EXPECT_TRUE(state.reference().isApprox(home));
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));
}

TEST(ArmControlState, ReleaseLatchPreservesSynchronizedReference)
{
  ArmControlState state("left");
  const ArmVector initial = ArmVector::Constant(0.2);
  state.initialize(initial);
  state.updateGripRequest(true, initial);
  ASSERT_TRUE(state.active());

  const ArmVector boundary_measured = ArmVector::Constant(0.3);
  const ArmVector latched_command = ArmVector::Constant(-0.4);
  state.enterHold(boundary_measured, true, "recording boundary");
  state.synchronizeReference(latched_command, "latched recording command");
  ASSERT_TRUE(state.releaseRequired());

  const ArmVector later_measured = ArmVector::Constant(0.5);
  state.updateGripRequest(false, later_measured);
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_FALSE(state.releaseRequired());
  EXPECT_TRUE(state.reference().isApprox(latched_command));
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));
}

TEST(ArmControlState, ActiveReferenceAccumulatesWithMeasuredTrackingBound)
{
  ArmControlState state("left");
  ArmVector measured = ArmVector::Constant(0.5);
  ArmVector q_min = ArmVector::Constant(-2.0);
  ArmVector q_max = ArmVector::Constant(2.0);
  ArmVector qdot = ArmVector::Constant(0.2);

  state.initialize(measured);
  state.updateGripRequest(true, measured);
  ASSERT_TRUE(state.active());

  ASSERT_TRUE(state.integrateReference(qdot, 0.1, measured, q_min, q_max, 0.35));
  EXPECT_TRUE(state.reference().isApprox(ArmVector::Constant(0.52)));

  ASSERT_TRUE(state.integrateReference(qdot, 0.1, measured, q_min, q_max, 0.35));
  EXPECT_TRUE(state.reference().isApprox(ArmVector::Constant(0.54)));
  EXPECT_TRUE(state.previousVelocity().isApprox(qdot));
}

TEST(ArmControlState, ActiveReferenceIsBoundedRelativeToMeasuredState)
{
  ArmControlState state("left");
  const ArmVector measured = ArmVector::Zero();
  const ArmVector q_min = ArmVector::Constant(-2.0);
  const ArmVector q_max = ArmVector::Constant(2.0);
  const ArmVector qdot = ArmVector::Ones();

  state.initialize(measured);
  state.updateGripRequest(true, measured);
  ASSERT_TRUE(state.active());

  for (int i = 0; i < 100; ++i) {
    ASSERT_TRUE(state.integrateReference(qdot, 0.02, measured, q_min, q_max, 0.35));
  }
  EXPECT_TRUE((state.reference().array() <= 0.35 + 1.0e-12).all());
  EXPECT_TRUE((state.reference().array() >= -1.0e-12).all());
}

TEST(ArmControlState, TimeoutRequiresReleaseBeforeReactivation)
{
  ArmControlState state("right");
  const ArmVector measured = ArmVector::Zero();
  state.initialize(measured);
  state.updateGripRequest(true, measured);
  ASSERT_TRUE(state.active());

  state.enterHold(measured, true, "target timeout");
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_TRUE(state.releaseRequired());

  state.updateGripRequest(true, measured);
  EXPECT_FALSE(state.active());

  state.updateGripRequest(false, measured);
  EXPECT_FALSE(state.releaseRequired());
  EXPECT_EQ(state.state(), ArmRunState::Hold);

  state.updateGripRequest(true, measured);
  EXPECT_TRUE(state.active());
}

TEST(ArmControlState, LimitRecoveryOwnsReferenceAcrossGripEdges)
{
  ArmControlState state("left");
  const ArmVector measured = ArmVector::Constant(0.2);
  state.initialize(measured);
  state.updateGripRequest(true, measured);
  ASSERT_TRUE(state.active());

  const ArmVector boundary = ArmVector::Constant(0.5);
  state.enterLimitRecovery(boundary, true, "measured position outside URDF range");
  ASSERT_TRUE(state.recovering());
  EXPECT_TRUE(state.reference().isApprox(boundary));

  const ArmVector inward = ArmVector::Constant(0.4);
  state.updateRecoveryReference(inward);
  state.updateGripRequest(false, measured);
  EXPECT_TRUE(state.recovering());
  EXPECT_TRUE(state.reference().isApprox(inward));

  state.finishLimitRecovery(false, "recovery complete");
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_FALSE(state.releaseRequired());
  EXPECT_TRUE(state.reference().isApprox(inward));
}

TEST(ArmControlState, PressedSafetyMarginRecoveryRequiresReleaseBeforeActive)
{
  ArmControlState state("right");
  const ArmVector measured = ArmVector::Constant(0.76);
  const ArmVector held_reference = ArmVector::Constant(0.75);
  const ArmVector inward_reference = ArmVector::Constant(0.70);
  state.initialize(measured);
  state.synchronizeReference(held_reference, "last held command");

  state.enterLimitRecovery(
    held_reference, true, "measured position outside ACTIVE safety range");
  ASSERT_TRUE(state.recovering());
  EXPECT_TRUE(state.releaseRequired());
  EXPECT_TRUE(state.reference().isApprox(held_reference));

  state.updateRecoveryReference(inward_reference);
  state.finishLimitRecovery(true, "joint limit recovery complete");
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_TRUE(state.releaseRequired());
  EXPECT_TRUE(state.reference().isApprox(inward_reference));

  // Keeping Grip pressed must not enter ACTIVE at the recovery boundary.
  state.updateGripRequest(true, measured);
  EXPECT_FALSE(state.active());
  EXPECT_TRUE(state.reference().isApprox(inward_reference));

  state.updateGripRequest(false, measured);
  EXPECT_FALSE(state.releaseRequired());
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_TRUE(state.reference().isApprox(inward_reference));

  state.updateGripRequest(true, measured);
  EXPECT_TRUE(state.active());
  EXPECT_TRUE(state.reference().isApprox(inward_reference));
}

TEST(ArmControlState, ActivationIsRejectedUntilGripRelease)
{
  ArmControlState state("right");
  const ArmVector measured = ArmVector::Constant(0.2);
  state.initialize(measured);

  state.rejectActivation(true, measured, "outside activation safety range");
  EXPECT_EQ(state.state(), ArmRunState::Hold);
  EXPECT_TRUE(state.releaseRequired());
  state.updateGripRequest(true, measured);
  EXPECT_FALSE(state.active());

  state.updateGripRequest(false, measured);
  EXPECT_FALSE(state.releaseRequired());
  state.updateGripRequest(true, measured);
  EXPECT_TRUE(state.active());
}

TEST(ArmControlState, ConsecutiveFailuresLatchFault)
{
  ArmControlState state("right");
  const ArmVector measured = ArmVector::Zero();
  state.initialize(measured);
  state.updateGripRequest(true, measured);

  state.recordSolveFailure(measured, 3, "QP failure");
  EXPECT_EQ(state.state(), ArmRunState::Active);
  EXPECT_TRUE(state.previousVelocity().isZero(0.0));

  state.recordSolveFailure(measured, 3, "QP failure");
  EXPECT_EQ(state.state(), ArmRunState::Active);

  state.recordSolveFailure(measured, 3, "QP failure");
  EXPECT_EQ(state.state(), ArmRunState::Fault);
  EXPECT_TRUE(state.releaseRequired());

  state.updateGripRequest(true, measured);
  EXPECT_EQ(state.state(), ArmRunState::Fault);

  state.updateGripRequest(false, measured);
  EXPECT_EQ(state.state(), ArmRunState::Fault);
  state.completeRecovery(measured);
  EXPECT_EQ(state.state(), ArmRunState::Hold);
}

}  // namespace
