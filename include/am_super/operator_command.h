#ifndef AM_SUPER_INCLUDE_OPERATOR_COMMAND_H_
#define AM_SUPER_INCLUDE_OPERATOR_COMMAND_H_

#include <brain_box_msgs/msg/operator_command.hpp>

/** Commands sent by the human operator to transition SuperStates through the standard flow.
 * https://automodality.atlassian.net/wiki/spaces/AMROS/pages/929234949/AMROS+System+States
 */
enum class OperatorCommand : std::uint8_t
{
  ARM = brain_box_msgs::msg::OperatorCommand::ARM,
  CANCEL = brain_box_msgs::msg::OperatorCommand::CANCEL,
  LAUNCH = brain_box_msgs::msg::OperatorCommand::LAUNCH,
  PAUSE= brain_box_msgs::msg::OperatorCommand::PAUSE,
  RESUME=brain_box_msgs::msg::OperatorCommand::RESUME,
  MANUAL=brain_box_msgs::msg::OperatorCommand::MANUAL,
  LANDED=brain_box_msgs::msg::OperatorCommand::LANDED,
  ABORT=brain_box_msgs::msg::OperatorCommand::ABORT,
  SHUTDOWN=brain_box_msgs::msg::OperatorCommand::SHUTDOWN,
};

#endif
