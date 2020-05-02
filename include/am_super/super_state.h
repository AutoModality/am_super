#ifndef AM_SUPER_INCLUDE_SUPER_STATE_H_
#define AM_SUPER_INCLUDE_SUPER_STATE_H_

#include <brain_box_msgs/VxState.h>

enum class SuperState : std::uint8_t
{
  OFF = brain_box_msgs::VxState::OFF,
  BOOTING = brain_box_msgs::VxState::BOOTING,
  READY = brain_box_msgs::VxState::READY,
  ARMING = brain_box_msgs::VxState::ARMING,
  ARMED = brain_box_msgs::VxState::ARMED,
  AUTO = brain_box_msgs::VxState::AUTO,
  SEMI_AUTO = brain_box_msgs::VxState::SEMI_AUTO,
  HOLD = brain_box_msgs::VxState::HOLD,
  ABORT = brain_box_msgs::VxState::ABORT,
  MANUAL = brain_box_msgs::VxState::MANUAL,
  SHUTDOWN = brain_box_msgs::VxState::SHUTDOWN,
  LAST_STATE = brain_box_msgs::VxState::LAST_STATE
};

#endif

//BOOTING_WAIT_ONLINE_OK = brain_box_msgs::VxState::BOOTING,
//BOOTING_WAIT_CONFIGURE_OK = brain_box_msgs::VxState::BOOTING_WAIT_ACTIVE_OK,
