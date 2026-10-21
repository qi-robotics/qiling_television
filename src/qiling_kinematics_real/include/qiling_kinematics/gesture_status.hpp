#pragma once

#include <cstdint>

namespace qiling_kinematics
{

// Internal arbitration topic /teleop/gesture_status. Only the IK node writes
// this status; the O6 adapter remains the only /handscmd writer.
enum class GestureStatus : std::uint8_t
{
  Idle = 0,
  Wave = 1,
  ShakeOpen = 2,
  ShakeGrip = 3,
  BaoquanPose = 4,
  ReturnOpen = 5,
  Fault = 6,
  Preflight = 7,  // Reserved for compatibility; runtime collision preflight is disabled.
};

}  // namespace qiling_kinematics
