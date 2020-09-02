#ifndef AM_SUPER_INCLUDE_OPERATOR_COMMAND_H_
#define AM_SUPER_INCLUDE_OPERATOR_COMMAND_H_

#include <brain_box_msgs/OperatorCommand.h>


/** Commands sent by the human operator to transition SuperStates through the standard flow.
 * https://automodality.atlassian.net/wiki/spaces/AMROS/pages/929234949/AMROS+System+States
 */
enum class OperatorCommand : std::uint8_t
{
  ARM = brain_box_msgs::OperatorCommand::ARM,
  LAUNCH = brain_box_msgs::OperatorCommand::LAUNCH,
  PAUSE= brain_box_msgs::OperatorCommand::PAUSE,
  RESUME=brain_box_msgs::OperatorCommand::RESUME,
  MANUAL=brain_box_msgs::OperatorCommand::MANUAL,
  LANDED=brain_box_msgs::OperatorCommand::LANDED,
  ABORT=brain_box_msgs::OperatorCommand::ABORT,
  SHUTDOWN=brain_box_msgs::OperatorCommand::SHUTDOWN,
};

#endif