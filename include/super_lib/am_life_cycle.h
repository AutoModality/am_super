#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_H_
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_H_

#include <string_view>

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticStatus.h>

#include <super_lib/am_stat_list.h>
#include <super_lib/am_life_cycle_types.h>
#include <super_lib/am_life_cycle_mediator.h>

namespace am
{
/**
 * Parent for all nodes wishing to report their state for collective management.  
 * The LifeCycle is generalized to represent all nodes regardless of application.
 * 
 * Each node reports is own state, but also receives commands requesting transition.
 * 
 * Implementing nodes should override methods appropriate to satisfy the needs of the node.
 * 
 * Read more about ROS2 LifeCycle and view the handy diagram.
 * 
 * https://automodality.atlassian.net/wiki/spaces/AMROS/pages/901546330/AM+Node+LifeCycle
 * 
 */
class AMLifeCycle
{
  public:
    static constexpr std::string_view BROADCAST_NODE_NAME = "";

  private:
    /* Variables to help seperate business logic from AMLifeCycle ROS */
    AMLifeCycleMediator life_cycle_mediator_;
    AMLifeCycleMediator::LifeCycleInfo life_cycle_info_;
    AMLifeCycleMediator::ThrottleInfo throttle_info_;


    /**The moment configuration is requested for this node. Used with 
     * max_configure_seconds_ to allow startup error tolerance.*/
    ros::Time configure_start_time_;

    void setState(const LifeCycleState state);

    /* if status is valid, then set this status to status */
    bool setStatus(const LifeCycleStatus status);

    void transition(std::string transition_name, LifeCycleState initial_state, LifeCycleState transition_state,
                    LifeCycleState new_state, std::function<void(void)> on_function);
    void doTransition(std::string transition_name, bool success, LifeCycleState success_state,
                      LifeCycleState failure_state);
  protected:
    std::string node_name_;

    /**Maximum time errors will be ignored during configuration. */ 
    int configure_tolerance_s;

    diagnostic_updater::Updater updater_;
    AMStatList stats_list_;

    ros::NodeHandle nh_;
    ros::Timer heartbeat_timer_;
    ros::Publisher state_pub_;
    ros::Subscriber lifecycle_sub_;

    /**
     * @brief Default constructor
     */
    AMLifeCycle();

    /**
     * @brief Virtual destructor
     */
    virtual ~AMLifeCycle();

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from INACTIVE to ACTIVE.
     */
    void activate();
    virtual void onActivate();
    void doActivate(bool success);

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from INACTIVE to UNCONFIGURED.
     */
    void cleanup();
    virtual void onCleanup();
    void doCleanup(bool success);

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from UNCONFIGURED to INACTIVE.
     */
    void configure();
    virtual void onConfigure();
    void doConfigure(bool success);
    
    /**
     * @brief true if configuring and within the time allowed to configure
     */
    bool withinConfigureTolerance();

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from ACTIVE to INACTIVE.
     */
    void deactivate();
    virtual void onDeactivate();
    void doDeactivate(bool success);

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from FINALIZED to power off.
     */
    virtual void destroy();
    virtual void onDestroy();
    void doDestroy(bool success);

    /**
     * @brief Function to be defined by the user.
     *        Called at any time and transitions to UNCONFIGURED or FINALIZED.
     */
    void error();
    virtual void onError();
    void doError(bool success);

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from INACTIVE to FINALIZED.
     */
    void shutdown();
    virtual void onShutdown();
    void doShutdown(bool success);

    virtual void addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw);
    virtual void heartbeatCB(const ros::TimerEvent& event);
    void sendNodeUpdate();
    void lifecycleCB(const brain_box_msgs::LifeCycleCommand::ConstPtr msg);

    /**Specific parts of the lifecycle where nodes have responsibilities.*/
    LifeCycleState getState() const;
    /**Simple indication of health */
    LifeCycleStatus getStatus() const;
    /** @brief string represenation of LifeCycleState*/
    const std::string_view& getStateName();
    /** @brief string representation of LifeCycleStatus*/
    const std::string_view& getStatusName();

    double getThrottleS() const;
    void setThrottleS(const double throttleS);
    double getThrottle();

    /**Providing consistency when logging the current state. */
    void logState();
    
};  // class AMLifeCycle

};  // namespace am

#endif
