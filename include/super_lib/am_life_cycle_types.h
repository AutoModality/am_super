#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_TYPES_H_
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_TYPES_H_

#include <cstdint>
#include <brain_box_msgs/LifeCycleState.h>

namespace am
{

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

enum class LifeCycleStatus : std::uint8_t
{
    OK = brain_box_msgs::LifeCycleState::STATUS_OK,
    WARN = brain_box_msgs::LifeCycleState::STATUS_WARN,
    ERROR = brain_box_msgs::LifeCycleState::STATUS_ERROR,
    LAST_STATUS = brain_box_msgs::LifeCycleState::STATUS_LAST
};

};
#endif
