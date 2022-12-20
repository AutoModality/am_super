#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_TYPES_H_
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_TYPES_H_

#include <cstdint>
#include <brain_box_msgs/msg/life_cycle_state.hpp>
#include <brain_box_msgs/msg/life_cycle_command.hpp>

namespace am
{
/**
 * state is the lifecycle state which is about startup, shutdown, and error handling
 */
enum class LifeCycleState : std::uint8_t
{
  INVALID = brain_box_msgs::msg::LifeCycleState::STATE_INVALID,
  UNCONFIGURED = brain_box_msgs::msg::LifeCycleState::STATE_UNCONFIGURED,
  INACTIVE = brain_box_msgs::msg::LifeCycleState::STATE_INACTIVE,
  ACTIVE = brain_box_msgs::msg::LifeCycleState::STATE_ACTIVE,
  FINALIZED = brain_box_msgs::msg::LifeCycleState::STATE_FINALIZED,
  CONFIGURING = brain_box_msgs::msg::LifeCycleState::STATE_CONFIGURING,
  CLEANING_UP = brain_box_msgs::msg::LifeCycleState::STATE_CLEANING_UP,
  SHUTTING_DOWN = brain_box_msgs::msg::LifeCycleState::STATE_SHUTTING_DOWN,
  ACTIVATING = brain_box_msgs::msg::LifeCycleState::STATE_ACTIVATING,
  DEACTIVATING = brain_box_msgs::msg::LifeCycleState::STATE_DEACTIVATING,
  ERROR_PROCESSING = brain_box_msgs::msg::LifeCycleState::STATE_ERROR_PROCESSING
};

/**
 * status of the functionality of the node (i.e. is it operating to spec)
 */
enum class LifeCycleStatus : std::uint8_t
{
  OK = brain_box_msgs::msg::LifeCycleState::STATUS_OK,
  WARN = brain_box_msgs::msg::LifeCycleState::STATUS_WARN,
  ERROR = brain_box_msgs::msg::LifeCycleState::STATUS_ERROR
};

/**
 * lifecycle commands to nodes to change state
 */
enum class LifeCycleCommand : std::uint8_t
{
  CREATE = brain_box_msgs::msg::LifeCycleCommand::COMMAND_CREATE,
  CONFIGURE = brain_box_msgs::msg::LifeCycleCommand::COMMAND_CONFIGURE,
  CLEANUP = brain_box_msgs::msg::LifeCycleCommand::COMMAND_CLEANUP,
  ACTIVATE = brain_box_msgs::msg::LifeCycleCommand::COMMAND_ACTIVATE,
  DEACTIVATE = brain_box_msgs::msg::LifeCycleCommand::COMMAND_DEACTIVATE,
  SHUTDOWN = brain_box_msgs::msg::LifeCycleCommand::COMMAND_SHUTDOWN,
  DESTROY = brain_box_msgs::msg::LifeCycleCommand::COMMAND_DESTROY,

  //FIXME: there should be no last command
  LAST_COMMAND = brain_box_msgs::msg::LifeCycleCommand::COMMAND_LAST
};

};  // namespace am
#endif
