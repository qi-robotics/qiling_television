#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "qiling_kinematics/gesture_status.hpp"
#include "qi/msg/hands_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace
{

constexpr std::size_t kFingerCount = 6;
constexpr std::uint8_t kLeftMask = 0x01;
constexpr std::uint8_t kRightMask = 0x02;

using Position = std::array<std::uint16_t, kFingerCount>;
using qiling_kinematics::GestureStatus;

bool readPositionParameter(
  rclcpp::Node & node, const char * name, const Position & defaults, Position & values)
{
  std::vector<int64_t> default_values(defaults.begin(), defaults.end());
  const auto parameter = node.declare_parameter<std::vector<int64_t>>(name, default_values);
  if (parameter.size() != kFingerCount) {
    RCLCPP_ERROR(
      node.get_logger(), "%s must contain exactly %zu values", name, kFingerCount);
    return false;
  }

  for (std::size_t i = 0; i < kFingerCount; ++i) {
    if (parameter[i] < 0 || parameter[i] > std::numeric_limits<std::uint8_t>::max()) {
      RCLCPP_ERROR(
        node.get_logger(), "%s[%zu] must be in [0, 255], got %ld",
        name, i, parameter[i]);
      return false;
    }
    values[i] = static_cast<std::uint16_t>(parameter[i]);
  }
  return true;
}

std::string formatPosition(const Position & values)
{
  std::ostringstream stream;
  stream << '[';
  for (std::size_t i = 0; i < kFingerCount; ++i) {
    if (i != 0) {
      stream << ", ";
    }
    stream << values[i];
  }
  stream << ']';
  return stream.str();
}

}  // namespace

class O6CommandAdapterNode final : public rclcpp::Node
{
public:
  O6CommandAdapterNode()
  : Node("qiling_o6_command_adapter")
  {
    declare_parameter("state_topic", std::string("/teleop/o6_trigger_state"));
    declare_parameter("command_topic", std::string("/handscmd"));
    declare_parameter(
      "home_complete_topic", std::string("/teleop/startup_home_complete"));
    declare_parameter("require_home_complete", true);
    declare_parameter("gesture_status_topic", std::string("/teleop/gesture_status"));
    declare_parameter("gesture_status_timeout_sec", 0.30);
    const double refresh_hz = declare_parameter<double>("command_refresh_hz", 10.0);
    if (!std::isfinite(refresh_hz) || refresh_hz < 1.0 || refresh_hz > 20.0) {
      throw std::runtime_error("command_refresh_hz must be in [1, 20]");
    }

    const Position default_open = {255, 104, 255, 255, 255, 255};
    const Position default_close = {101, 60, 0, 0, 0, 0};
    if (!readPositionParameter(*this, "open_position", default_open, open_position_) ||
      !readPositionParameter(*this, "close_position", default_close, close_position_) ||
      !readPositionParameter(
        *this, "gesture_shake_right_position", Position{115, 115, 204, 204, 204, 204},
        gesture_shake_right_position_) ||
      !readPositionParameter(
        *this, "gesture_baoquan_right_position", Position{102, 102, 0, 0, 0, 0},
        gesture_baoquan_right_position_))
    {
      throw std::runtime_error("invalid O6 position parameter");
    }

    speed_ = declare_parameter<int>("speed", 200);
    if (speed_ < 0 || speed_ > std::numeric_limits<std::uint8_t>::max()) {
      throw std::runtime_error("speed must be in [0, 255]");
    }

    const auto state_topic = get_parameter("state_topic").as_string();
    const auto command_topic = get_parameter("command_topic").as_string();
    const auto home_topic = get_parameter("home_complete_topic").as_string();

    command_pub_ = create_publisher<qi::msg::HandsCmd>(command_topic, rclcpp::QoS(1));
    state_sub_ = create_subscription<std_msgs::msg::UInt8>(
      state_topic, rclcpp::QoS(1),
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        onTriggerState(message->data);
      });
    home_sub_ = create_subscription<std_msgs::msg::Bool>(
      home_topic, rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (home_status_received_ && home_complete_ == message->data) {
          return;
        }
        home_status_received_ = true;
        home_complete_ = message->data;
        RCLCPP_INFO(
          get_logger(), "startup home complete=%s", home_complete_ ? "true" : "false");
      });
    gesture_sub_ = create_subscription<std_msgs::msg::UInt8>(
      get_parameter("gesture_status_topic").as_string(), rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        onGestureStatus(message->data);
      });
    refresh_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / refresh_hz)),
      std::bind(&O6CommandAdapterNode::refreshCommand, this));

    RCLCPP_INFO(
      get_logger(),
      "O6 command adapter ready: state=%s, command=%s, speed=%d, "
      "hold-last-target-on-input-loss=true, refresh=%.1f Hz",
      state_topic.c_str(), command_topic.c_str(), speed_, refresh_hz);
    RCLCPP_INFO(
      get_logger(), "open=%s, close=%s",
      formatPosition(open_position_).c_str(), formatPosition(close_position_).c_str());
  }

private:
  void onGestureStatus(std::uint8_t raw)
  {
    gesture_status_received_ = true;
    last_gesture_status_time_ = std::chrono::steady_clock::now();
    const GestureStatus next = raw <= static_cast<std::uint8_t>(GestureStatus::Preflight) ?
      static_cast<GestureStatus>(raw) : GestureStatus::Fault;
    if (next == gesture_status_) {
      return;
    }
    const GestureStatus previous = gesture_status_;
    gesture_status_ = next;
    if (next == GestureStatus::Idle) {
      if (previous != GestureStatus::Idle) {
        trigger_rearm_mask_ = kLeftMask | kRightMask;
        have_state_ = false;
      }
      return;
    }
    trigger_rearm_mask_ = kLeftMask | kRightMask;
    if (next == GestureStatus::Preflight || next == GestureStatus::Fault) {
      return; // Retain the last physical hand target while motion is locked.
    }
    if (next == GestureStatus::ShakeGrip) {
      publishPositions(open_position_, gesture_shake_right_position_, "gesture shake grip");
    } else if (next == GestureStatus::BaoquanPose) {
      publishPositions(open_position_, gesture_baoquan_right_position_, "gesture baoquan pose");
    } else if (next == GestureStatus::ReturnOpen && previous == GestureStatus::BaoquanPose) {
      publishPositions(open_position_, open_position_, "gesture baoquan hands open");
    } else {
      publishPositions(open_position_, open_position_, "gesture hands open");
    }
  }

  void onTriggerState(std::uint8_t raw_state)
  {
    std::uint8_t state = raw_state & (kLeftMask | kRightMask);
    if (!gesture_status_received_ ||
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_gesture_status_time_).count() >
      get_parameter("gesture_status_timeout_sec").as_double())
    {
      return;
    }
    if (gesture_status_ != GestureStatus::Idle) {
      return;
    }
    if (get_parameter("require_home_complete").as_bool() && !home_complete_) {
      return;
    }
    // Each hand rearms on its own release after an action. A held trigger on
    // one controller must not block the other controller's fingers.
    trigger_rearm_mask_ &= state;
    state &= static_cast<std::uint8_t>(~trigger_rearm_mask_);

    if (!have_state_) {
      have_state_ = true;
      last_state_ = state;
      publishCommand(state, "initial O6 state");
      return;
    }

    const std::uint8_t changed = state ^ last_state_;
    if (changed == 0) {
      return;
    }

    last_state_ = state;
    publishCommand(state, "O6 trigger state changed");
  }

  void publishCommand(std::uint8_t state, const char * reason)
  {
    const Position & left_position = (state & kLeftMask) != 0 ? close_position_ : open_position_;
    const Position & right_position = (state & kRightMask) != 0 ? close_position_ : open_position_;
    publishPositions(left_position, right_position, reason);
  }

  void publishPositions(
    const Position & left_position, const Position & right_position, const char * reason)
  {
    qi::msg::HandsCmd command;
    command.mode = 1;
    command.mode_ctrl = 1;
    command.timestamp = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());

    for (std::size_t i = 0; i < kFingerCount; ++i) {
      command.hands[0].positions[i] = left_position[i];
      command.hands[0].durations[i] = static_cast<std::uint16_t>(speed_);
      command.hands[1].positions[i] = right_position[i];
      command.hands[1].durations[i] = static_cast<std::uint16_t>(speed_);
    }
    command.hands[0].mode = 1;
    command.hands[0].hand_id = 0;
    command.hands[1].mode = 1;
    command.hands[1].hand_id = 1;

    command_pub_->publish(command);
    last_left_position_ = left_position;
    last_right_position_ = right_position;
    have_command_target_ = true;
    if (reason != nullptr) {
      if (command_pub_->get_subscription_count() == 0) {
        RCLCPP_WARN(
          get_logger(), "no /handscmd subscriber discovered; O6 command may not be received");
      }
      RCLCPP_INFO(
        get_logger(),
        "%s: left=%s, right=%s, speed=%d",
        reason, formatPosition(left_position).c_str(), formatPosition(right_position).c_str(), speed_);
    }
  }

  void refreshCommand()
  {
    if (!have_command_target_ || !gesture_status_received_ || !home_complete_ ||
      gesture_status_ == GestureStatus::Fault ||
      gesture_status_ == GestureStatus::Preflight ||
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_gesture_status_time_).count() >
      get_parameter("gesture_status_timeout_sec").as_double())
    {
      return;
    }
    publishPositions(last_left_position_, last_right_position_, nullptr);
  }

  Position open_position_{};
  Position close_position_{};
  Position gesture_shake_right_position_{};
  Position gesture_baoquan_right_position_{};
  int speed_{200};
  bool home_complete_{false};
  bool home_status_received_{false};
  bool have_state_{false};
  std::uint8_t trigger_rearm_mask_{kLeftMask | kRightMask};
  bool gesture_status_received_{false};
  bool have_command_target_{false};
  GestureStatus gesture_status_{GestureStatus::Idle};
  Position last_left_position_{};
  Position last_right_position_{};
  std::chrono::steady_clock::time_point last_gesture_status_time_{};
  std::uint8_t last_state_{0};
  rclcpp::Publisher<qi::msg::HandsCmd>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr home_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gesture_sub_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<O6CommandAdapterNode>());
  } catch (const std::exception & error) {
    std::cerr << "O6 command adapter failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
