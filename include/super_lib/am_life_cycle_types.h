#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_TYPES_H_
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_TYPES_H_

#include <cstdint>
#include <brain_box_msgs/LifeCycleState.h>
#include <brain_box_msgs/LifeCycleCommand.h>

namespace am
{
/**
 * state is the lifecycle state which is about startup, shutdown, and error handling
 */
enum class LifeCycleState : std::uint8_t
{
  INVALID = brain_box_msgs::LifeCycleState::STATE_INVALID,
  UNCONFIGURED = brain_box_msgs::LifeCycleState::STATE_UNCONFIGURED,
  INACTIVE = brain_box_msgs::LifeCycleState::STATE_INACTIVE,
  ACTIVE = brain_box_msgs::LifeCycleState::STATE_ACTIVE,
  FINALIZED = brain_box_msgs::LifeCycleState::STATE_FINALIZED,
  CONFIGURING = brain_box_msgs::LifeCycleState::STATE_CONFIGURING,
  CLEANING_UP = brain_box_msgs::LifeCycleState::STATE_CLEANING_UP,
  SHUTTING_DOWN = brain_box_msgs::LifeCycleState::STATE_SHUTTING_DOWN,
  ACTIVATING = brain_box_msgs::LifeCycleState::STATE_ACTIVATING,
  DEACTIVATING = brain_box_msgs::LifeCycleState::STATE_DEACTIVATING,
  ERROR_PROCESSING = brain_box_msgs::LifeCycleState::STATE_ERROR_PROCESSING,
  LAST_STATE = brain_box_msgs::LifeCycleState::STATE_LAST
};

/**
 * status of the functionality of the node (i.e. is it operating to spec)
 */
enum class LifeCycleStatus : std::uint8_t
{OK = brain_box_msgs::LifeCycleState::STATUS_OK,
  WARN = brain_box_msgs::LifeCycleState::STATUS_WARN,
  ERROR = brain_box_msgs::LifeCycleState::STATUS_ERROR,
  LAST_STATUS = brain_box_msgs::LifeCycleState::STATUS_LAST
};

/**
 * lifecycle commands to nodes to change state
 */
enum class LifeCycleCommand : std::uint8_t
{
  CREATE = brain_box_msgs::LifeCycleCommand::COMMAND_CREATE,
  CONFIGURE = brain_box_msgs::LifeCycleCommand::COMMAND_CONFIGURE,
  CLEANUP = brain_box_msgs::LifeCycleCommand::COMMAND_CLEANUP,
  ACTIVATE = brain_box_msgs::LifeCycleCommand::COMMAND_ACTIVATE,
  DEACTIVATE = brain_box_msgs::LifeCycleCommand::COMMAND_DEACTIVATE,
  SHUTDOWN = brain_box_msgs::LifeCycleCommand::COMMAND_SHUTDOWN,
  DESTROY = brain_box_msgs::LifeCycleCommand::COMMAND_DESTROY,
  LAST_COMMAND = brain_box_msgs::LifeCycleCommand::COMMAND_LAST
};

};  // namespace am
#endif
