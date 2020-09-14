#ifndef AM_SUPER_INCLUDE_CONTROLLER_STATE_H_
#define AM_SUPER_INCLUDE_CONTROLLER_STATE_H_

#include <brain_box_msgs/ControllerState.h>

/** State of flight controller
 * https://automodality.atlassian.net/wiki/spaces/AMROS/pages/929234949/AMROS+System+States
 */
enum class ControllerState : std::uint8_t
{
  COMPLETED = brain_box_msgs::ControllerState::COMPLETED
};

#endif