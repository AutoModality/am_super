#ifndef AM_SUPER_INCLUDE_SUPER_LIFECYCLE_H_
#define AM_SUPER_INCLUDE_SUPER_LIFECYCLE_H_

#include <cstdint>

#include <ros/ros.h>

#include <brain_box_msgs/LifeCycleState.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

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

class AMLifeCycle
{
protected:
    LifeCycleState state_;
    LifeCycleStatus status_;

    ros::NodeHandle nh_;
    ros::Timer heartbeat_timer_;
    ros::Publisher state_pub_;

    /**
     * @brief Default constructor
     */
    AMLifeCycle();

    /**
     * @brief Virtual destructor
     */
    virtual ~AMLifeCycle();

    virtual void configure();
    virtual void cleanup();
    virtual void activate();
    virtual void deactivate();
    virtual void shutdown();
    virtual void destroy();

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from UNCONFIGURED to INACTIVE.
     */
    virtual void onConfigure();

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from INACTIVE to UNCONFIGURED.
     */
    virtual void onCleanUp();

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from INACTIVE to ACTIVE.
     */
    virtual void onActivate();

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from ACTIVE to INACTIVE.
     */
    virtual void onDeactivate();

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from INACTIVE to FINALIZED.
     */
    virtual void onShutdown();

    /**
     * @brief Function to be defined by the user.
     *        Called after an error and may transition to UNCONFIGURED or FINALIZED.
     */
    virtual void onError();

    virtual void heartbeatCB(const ros::TimerEvent &event);

};
// class AMLifeCycle

}
;
// namespace am

#endif
