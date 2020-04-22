#ifndef AM_SUPER_INCLUDE_SUPER_STATE_H_
#define AM_SUPER_INCLUDE_SUPER_STATE_H_

#include <brain_box_msgs/VxState.h>

enum class SuperState : std::uint8_t
{
	UNKNOWN			= brain_box_msgs::VxState::UNKNOWN,
	BOOTING			= brain_box_msgs::VxState::BOOTING,
	READY			= brain_box_msgs::VxState::READY,
	ARMED			= brain_box_msgs::VxState::ARMED,
	AUTO_FEATURE	= brain_box_msgs::VxState::AUTO_FEATURE,
	AUTO_NO_FEATURE	= brain_box_msgs::VxState::AUTO_NO_FEATURE,
	HOLD			= brain_box_msgs::VxState::HOLD,
	ERROR			= brain_box_msgs::VxState::ERROR,
	MANUAL			= brain_box_msgs::VxState::MANUAL,
    LAST_STATE 		= brain_box_msgs::VxState::LAST_STATE
};

#endif
