#ifndef AM_SUPER_INCLUDE_SUPER_STATE_H_
#define AM_SUPER_INCLUDE_SUPER_STATE_H_

#include <brain_box_msgs/msg/vx_state.hpp>

enum class SuperState : std::uint8_t
{
  OFF = brain_box_msgs::msg::VxState::OFF,
  BOOTING = brain_box_msgs::msg::VxState::BOOTING,
  READY = brain_box_msgs::msg::VxState::READY,
  ARMING = brain_box_msgs::msg::VxState::ARMING,
  ARMED = brain_box_msgs::msg::VxState::ARMED,
  AUTO = brain_box_msgs::msg::VxState::AUTO,
  DISARMING = brain_box_msgs::msg::VxState::DISARMING,
  SEMI_AUTO = brain_box_msgs::msg::VxState::SEMI_AUTO,
  HOLD = brain_box_msgs::msg::VxState::HOLD,
  ABORT = brain_box_msgs::msg::VxState::ABORT,
  MANUAL = brain_box_msgs::msg::VxState::MANUAL,
  SHUTDOWN = brain_box_msgs::msg::VxState::SHUTDOWN,
};

#endif

// BOOTING_WAIT_ONLINE_OK = brain_box_msgs::VxState::BOOTING,
// BOOTING_WAIT_CONFIGURE_OK = brain_box_msgs::VxState::BOOTING_WAIT_ACTIVE_OK,
