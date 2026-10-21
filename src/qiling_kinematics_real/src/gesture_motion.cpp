#include "qiling_kinematics/gesture_motion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <pinocchio/collision/collision.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <yaml-cpp/yaml.h>

namespace qiling_kinematics
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kStep = 0.02;
constexpr int kLeft = 0;
constexpr int kRight = 1;
constexpr std::array<const char *, kSingleArmDof> kJointParts = {
  "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
  "wrist_roll", "wrist_pitch", "wrist_yaw"};

ArmVector readArm(const YAML::Node & node, const std::string & label)
{
  if (!node.IsSequence() || node.size() != kSingleArmDof) {
    throw std::runtime_error(label + " must contain seven joint angles");
  }
  ArmVector result;
  for (int i = 0; i < kSingleArmDof; ++i) {
    result[i] = node[i].as<double>();
  }
  if (!result.allFinite()) {
    throw std::runtime_error(label + " contains non-finite angle");
  }
  return result;
}

double readPositive(const YAML::Node & node, const std::string & label)
{
  const double value = node.as<double>();
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(label + " must be finite and positive");
  }
  return value;
}

double quintic(double u)
{
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

double quinticDerivative(double u)
{
  return 30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u;
}

double quinticSecond(double u)
{
  return 60.0 * u - 180.0 * u * u + 120.0 * u * u * u;
}

bool isArmLink(const std::string & name, const char * side)
{
  for (const char * part : kJointParts) {
    if (name == std::string(side) + "_" + part + "_link_0") {
      return true;
    }
  }
  return name == (std::string(side) == "left" ? "LH_hand_base_link_0" : "RH_hand_base_link_0");
}

bool isFixedBody(const std::string & name)
{
  if (name == "base_link_0") {
    return true;
  }
  for (const char * side : {"left", "right"}) {
    for (const char * part : {"hip_", "knee_", "foot_"}) {
      if (name.rfind(std::string(side) + "_" + part, 0) == 0) {
        return true;
      }
    }
  }
  return false;
}
}  // namespace

const char * gestureName(GestureKind kind)
{
  switch (kind) {
    case GestureKind::Wave: return "wave";
    case GestureKind::Shake: return "shake";
    case GestureKind::Baoquan: return "baoquan";
    case GestureKind::Home: return "home";
  }
  return "unknown";
}

GesturePlan GesturePlan::load(
  const std::filesystem::path & config_path, GestureKind kind,
  const std::array<ArmVector, 2> & start,
  const std::array<ArmVector, 2> & home,
  const std::array<ArmVector, 2> & lower,
  const std::array<ArmVector, 2> & upper,
  const ArmVector & speed_limit,
  const ArmVector & acceleration_limit)
{
  const YAML::Node root = YAML::LoadFile(config_path.string());
  const YAML::Node order = root["joint_order"];
  if (!order.IsSequence() || order.size() != kSingleArmDof) {
    throw std::runtime_error("gesture joint_order must contain seven entries");
  }
  for (int i = 0; i < kSingleArmDof; ++i) {
    if (order[i].as<std::string>() != kJointParts[i]) {
      throw std::runtime_error("gesture joint_order differs from motor/kinematics order");
    }
  }
  if (root["motor_offsets"]["left"].as<int>() != 12 ||
    root["motor_offsets"]["right"].as<int>() != 19)
  {
    throw std::runtime_error("gesture motor offsets must be left=12, right=19");
  }
  const YAML::Node spec = root["actions"][gestureName(kind)];
  if (!spec.IsMap()) {
    throw std::runtime_error("gesture action is absent from config");
  }
  GesturePlan plan;
  plan.kind_ = kind;
  plan.start_ = start;
  plan.target_ = start;
  plan.home_ = home;
  if (kind == GestureKind::Home) {
    // A home chord moves directly from the last published arm command to home.
    plan.target_ = home;
  }
  plan.lower_ = lower;
  plan.upper_ = upper;
  plan.speed_limit_ = speed_limit;
  plan.acceleration_limit_ = acceleration_limit;
  plan.margin_ = readPositive(root["minimum_limit_clearance_rad"], "minimum_limit_clearance_rad");
  plan.safety_factor_ = readPositive(root["duration_safety_factor"], "duration_safety_factor");
  if (plan.safety_factor_ < 1.0) {
    throw std::runtime_error("duration_safety_factor must be >= 1");
  }
  plan.cycles_ = readPositive(spec["motion_cycles"], "motion_cycles");
  for (int side = 0; side < 2; ++side) {
    const char * side_name = side == kLeft ? "left" : "right";
    const YAML::Node target_node = spec[std::string(side_name) + "_target"];
    if (target_node) {
      plan.target_[side] = readArm(target_node, std::string(side_name) + "_target");
    }
    const YAML::Node oscillation = spec[std::string(side_name) + "_oscillation"];
    if (oscillation) {
      if (!oscillation.IsMap()) {
        throw std::runtime_error("oscillation must be a joint map");
      }
      for (const auto & joint : oscillation) {
        const std::string name = joint.first.as<std::string>();
        const auto found = std::find_if(
          kJointParts.begin(), kJointParts.end(),
          [&name](const char * item) {return name == item;});
        if (found == kJointParts.end() || !joint.second.IsSequence()) {
          throw std::runtime_error("invalid oscillation joint: " + name);
        }
        const int index = static_cast<int>(std::distance(kJointParts.begin(), found));
        for (const auto & term : joint.second) {
          if (!term.IsSequence() || term.size() != 2) {
            throw std::runtime_error("oscillation term must be [amplitude, phase]");
          }
          const double amplitude = term[0].as<double>();
          const double phase = term[1].as<double>();
          if (!std::isfinite(amplitude) || !std::isfinite(phase)) {
            throw std::runtime_error("oscillation term must be finite");
          }
          plan.terms_[side][index].push_back({amplitude, phase});
        }
      }
    }
  }
  if (kind == GestureKind::Shake) {
    plan.grip_wait_sec_ = spec["grip_wait_sec"] ?
      readPositive(spec["grip_wait_sec"], "grip_wait_sec") : 1.0;
  }
  if (!speed_limit.allFinite() || !acceleration_limit.allFinite() ||
    (speed_limit.array() <= 0).any() || (acceleration_limit.array() <= 0).any())
  {
    throw std::runtime_error("gesture velocity/acceleration limits must be positive");
  }
  plan.durations_[0] = plan.retimedDuration(
    GesturePhase::Entry, readPositive(spec["entry_min_sec"], "entry_min_sec"));
  plan.durations_[1] = plan.retimedDuration(
    GesturePhase::Motion, readPositive(spec["motion_min_sec"], "motion_min_sec"));
  plan.durations_[2] = plan.retimedDuration(
    GesturePhase::ReturnHome, readPositive(spec["return_min_sec"], "return_min_sec"));
  plan.validate();
  return plan;
}

GestureFrame GesturePlan::segmentSample(GesturePhase phase, double u, double duration) const
{
  GestureFrame frame;
  frame.phase = phase;
  frame.velocity = {ArmVector::Zero(), ArmVector::Zero()};
  frame.acceleration = {ArmVector::Zero(), ArmVector::Zero()};
  const double s = std::clamp(u, 0.0, 1.0);
  if (phase == GesturePhase::Entry || phase == GesturePhase::ReturnHome) {
    const auto & from = phase == GesturePhase::Entry ? start_ : target_;
    const auto & to = phase == GesturePhase::Entry ? target_ : home_;
    for (int side = 0; side < 2; ++side) {
      const ArmVector delta = to[side] - from[side];
      frame.position[side] = from[side] + quintic(s) * delta;
      frame.velocity[side] = quinticDerivative(s) / duration * delta;
      frame.acceleration[side] = quinticSecond(s) / (duration * duration) * delta;
    }
    return frame;
  }
  frame.position = target_;
  if (phase == GesturePhase::GripWait) {
    return frame;
  }
  const double sn = std::sin(kPi * s);
  const double cs = std::cos(kPi * s);
  const double envelope = std::pow(sn, 4);
  const double envelope_d = 4 * kPi * sn * sn * sn * cs;
  const double envelope_dd = 4 * kPi * kPi * (3 * sn * sn * cs * cs - std::pow(sn, 4));
  const double phase_rate = 2 * kPi * cycles_;
  for (int side = 0; side < 2; ++side) {
    for (int joint = 0; joint < kSingleArmDof; ++joint) {
      for (const auto & term : terms_[side][joint]) {
        const double angle = phase_rate * s + term.phase;
        frame.position[side][joint] += term.amplitude * envelope * std::sin(angle);
        frame.velocity[side][joint] += term.amplitude / duration *
          (envelope_d * std::sin(angle) + envelope * phase_rate * std::cos(angle));
        frame.acceleration[side][joint] += term.amplitude / (duration * duration) *
          (envelope_dd * std::sin(angle) +
          2 * envelope_d * phase_rate * std::cos(angle) -
          envelope * phase_rate * phase_rate * std::sin(angle));
      }
    }
  }
  return frame;
}

double GesturePlan::retimedDuration(GesturePhase phase, double minimum) const
{
  double duration = minimum;
  for (int step = 0; step <= 2000; ++step) {
    const double s = step / 2000.0;
    const double d = quinticDerivative(s);
    const double dd = quinticSecond(s);
    const double sn = std::sin(kPi * s);
    const double cs = std::cos(kPi * s);
    const double envelope = std::pow(sn, 4);
    const double envelope_d = 4 * kPi * sn * sn * sn * cs;
    const double envelope_dd = 4 * kPi * kPi * (3 * sn * sn * cs * cs - std::pow(sn, 4));
    const double rate = 2 * kPi * cycles_;
    for (int side = 0; side < 2; ++side) {
      const ArmVector delta = phase == GesturePhase::Entry ?
        target_[side] - start_[side] : home_[side] - target_[side];
      for (int joint = 0; joint < kSingleArmDof; ++joint) {
        double first = phase == GesturePhase::Motion ? 0.0 : d * delta[joint];
        double second = phase == GesturePhase::Motion ? 0.0 : dd * delta[joint];
        if (phase == GesturePhase::Motion) {
          for (const auto & term : terms_[side][joint]) {
            const double angle = rate * s + term.phase;
            first += term.amplitude *
              (envelope_d * std::sin(angle) + envelope * rate * std::cos(angle));
            second += term.amplitude *
              (envelope_dd * std::sin(angle) + 2 * envelope_d * rate * std::cos(angle)
              - envelope * rate * rate * std::sin(angle));
          }
        }
        duration = std::max(duration, std::abs(first) / speed_limit_[joint]);
        duration = std::max(duration, std::sqrt(std::abs(second) / acceleration_limit_[joint]));
      }
    }
  }
  return std::ceil(duration * safety_factor_ / kStep) * kStep;
}

void GesturePlan::validate() const
{
  const double boundaries[] = {0.0, durations_[0], durations_[0] + grip_wait_sec_,
    durations_[0] + grip_wait_sec_ + durations_[1], totalDuration()};
  for (double elapsed = 0.0; elapsed <= totalDuration() + 1e-9; elapsed += kStep) {
    const GestureFrame frame = sample(elapsed);
    for (int side = 0; side < 2; ++side) {
      if (!frame.position[side].allFinite() || !frame.velocity[side].allFinite() ||
        !frame.acceleration[side].allFinite())
      {
        throw std::runtime_error("gesture trajectory has non-finite sample");
      }
      for (int joint = 0; joint < kSingleArmDof; ++joint) {
        const double value = frame.position[side][joint];
        // Home may start inside the configured soft clearance. Its quintic
        // entry moves monotonically from that in-range start toward a home
        // position inside the clearance, so enforce the URDF hard range until
        // the entry has reached home. All other phases/actions retain margin_.
        const double clearance =
          kind_ == GestureKind::Home && frame.phase == GesturePhase::Entry ? 0.0 : margin_;
        if (value < lower_[side][joint] + clearance - 1e-9 ||
          value > upper_[side][joint] - clearance + 1e-9 ||
          std::abs(frame.velocity[side][joint]) > speed_limit_[joint] + 1e-9 ||
          std::abs(frame.acceleration[side][joint]) > acceleration_limit_[joint] + 1e-9)
        {
          std::ostringstream out;
          out << "gesture limit/speed violation at t=" << elapsed << " side=" << side
              << " joint=" << kJointParts[joint];
          throw std::runtime_error(out.str());
        }
      }
    }
  }
  // Catch phase boundaries even when they fall between 50 Hz samples.
  for (double elapsed : boundaries) {
    const GestureFrame frame = sample(elapsed);
    for (int side = 0; side < 2; ++side) {
      if (!frame.position[side].allFinite()) {
        throw std::runtime_error("non-finite gesture phase boundary");
      }
    }
  }
}

GestureFrame GesturePlan::sample(double elapsed_sec) const
{
  if (!std::isfinite(elapsed_sec)) {
    throw std::runtime_error("gesture sample time must be finite");
  }
  double t = std::max(elapsed_sec, 0.0);
  if (t < durations_[0]) {
    return segmentSample(GesturePhase::Entry, t / durations_[0], durations_[0]);
  }
  t -= durations_[0];
  if (t < grip_wait_sec_) {
    return segmentSample(GesturePhase::GripWait, 0.0, 1.0);
  }
  t -= grip_wait_sec_;
  if (t < durations_[1]) {
    return segmentSample(GesturePhase::Motion, t / durations_[1], durations_[1]);
  }
  t -= durations_[1];
  if (t < durations_[2]) {
    return segmentSample(GesturePhase::ReturnHome, t / durations_[2], durations_[2]);
  }
  GestureFrame frame;
  frame.phase = GesturePhase::Complete;
  frame.position = home_;
  frame.velocity = {ArmVector::Zero(), ArmVector::Zero()};
  frame.acceleration = {ArmVector::Zero(), ArmVector::Zero()};
  return frame;
}

double GesturePlan::totalDuration() const
{
  return durations_[0] + grip_wait_sec_ + durations_[1] + durations_[2];
}

std::string checkGestureCollisions(
  const GesturePlan & plan,
  const std::filesystem::path & urdf_path,
  const std::filesystem::path & package_parent)
{
  try {
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path.string(), model);
    pinocchio::GeometryModel geom;
    pinocchio::urdf::buildGeom(
      model, urdf_path.string(), pinocchio::COLLISION, geom,
      std::vector<std::string>{package_parent.string()});
    std::array<std::array<int, 8>, 2> arm_geom{};
    std::array<int, 2> counts{};
    std::vector<int> fixed_geom;
    for (std::size_t i = 0; i < geom.geometryObjects.size(); ++i) {
      const auto & name = geom.geometryObjects[i].name;
      if (isFixedBody(name)) {
        fixed_geom.push_back(static_cast<int>(i));
      }
      for (int side = 0; side < 2; ++side) {
        if (isArmLink(name, side == kLeft ? "left" : "right")) {
          if (counts[side] >= 8) {
            return "unexpected duplicate arm collision mesh";
          }
          arm_geom[side][counts[side]++] = static_cast<int>(i);
        }
      }
    }
    if (counts[kLeft] != 8 || counts[kRight] != 8 || fixed_geom.empty()) {
      return "missing expected arm or fixed-body collision mesh";
    }
    for (int side = 0; side < 2; ++side) {
      for (int first = 0; first < 8; ++first) {
        for (int second = first + 3; second < 8; ++second) {
          geom.addCollisionPair(pinocchio::CollisionPair(
              arm_geom[side][first], arm_geom[side][second]));
        }
        for (int fixed : fixed_geom) {
          if (geom.geometryObjects[fixed].name == "base_link_0" && first == 0) {
            continue; // attached shoulder mounting mesh
          }
          geom.addCollisionPair(pinocchio::CollisionPair(fixed, arm_geom[side][first]));
        }
      }
    }
    for (int left : arm_geom[kLeft]) {
      for (int right : arm_geom[kRight]) {
        geom.addCollisionPair(pinocchio::CollisionPair(left, right));
      }
    }
    std::array<std::array<int, kSingleArmDof>, 2> q_index{};
    for (int side = 0; side < 2; ++side) {
      const char * name = side == kLeft ? "left" : "right";
      for (int joint = 0; joint < kSingleArmDof; ++joint) {
        const auto joint_id = model.getJointId(std::string(name) + "_" + kJointParts[joint] + "_joint");
        if (joint_id == 0 || model.nqs[joint_id] != 1) {
          return "URDF joint mapping mismatch during collision preflight";
        }
        q_index[side][joint] = model.idx_qs[joint_id];
      }
    }
    pinocchio::Data data(model);
    pinocchio::GeometryData geom_data(geom);
    Eigen::VectorXd q = pinocchio::neutral(model);
    for (double t = 0.0; t <= plan.totalDuration() + 1e-9; t += kStep) {
      const auto frame = plan.sample(t);
      for (int side = 0; side < 2; ++side) {
        for (int joint = 0; joint < kSingleArmDof; ++joint) {
          q[q_index[side][joint]] = frame.position[side][joint];
        }
      }
      if (pinocchio::computeCollisions(model, data, geom, geom_data, q, true)) {
        for (std::size_t pair_id = 0; pair_id < geom.collisionPairs.size(); ++pair_id) {
          if (geom_data.collisionResults[pair_id].isCollision()) {
            const auto & pair = geom.collisionPairs[pair_id];
            std::ostringstream out;
            out << "t=" << t << " collision "
                << geom.geometryObjects[pair.first].name << " / "
                << geom.geometryObjects[pair.second].name;
            return out.str();
          }
        }
        return "collision detected with unknown pair";
      }
    }
    return {};
  } catch (const std::exception & error) {
    return std::string("collision preflight unavailable: ") + error.what();
  }
}

}  // namespace qiling_kinematics
