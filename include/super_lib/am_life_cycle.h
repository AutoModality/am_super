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
    
    //internal methods called to begin the transition.  See on* for corresponding definitions.
    void configure();
    void activate();
    void deactivate();
    void shutdown();
    void destroy();
    void cleanup();
    void sendNodeUpdate();

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

    template<typename T>

    /** Exactly like ros::param, but logs INFO level showing the actual value assigned. 
     */
    bool param(const std::string& param_name, T& param_val, const T& default_val) const;

    //on* overriden by implementing node to do what is needed to satisfy the objective of the method
    //do* is called by the implementing node when the objective attempt has completed and status is to be reported
    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from INACTIVE to ACTIVE.
     */
    virtual void onActivate();
    void doActivate(bool success);

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from INACTIVE to UNCONFIGURED.
     */
    virtual void onCleanup();
    void doCleanup(bool success);

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from UNCONFIGURED to INACTIVE.
     */
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
    virtual void onDeactivate();
    void doDeactivate(bool success);

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from FINALIZED to power off.
     */
    virtual void onDestroy();
    void doDestroy(bool success);

    /**
     * Called by all when an error has happened.  Will set the status to ERROR and state to ERROR_PROCESSING
     * which will eventually lead to FINALIZED.
     * @param error_code provides a reference for the developer to correlate log output to the originating error.
     * @param forced terminal error that will not allow any tolerance
     */
    void error(std::string error_code="NNLW",bool forced = false);

    /**
     * @brief Function to be defined by the user.
     *        Called at any time and transitions to UNCONFIGURED or FINALIZED.
     */
    virtual void onError();
    void doError(bool success);

    /**
     * @brief Function to be defined by the user.
     *        Called at the end of transition from INACTIVE to FINALIZED.
     */
    virtual void onShutdown();
    void doShutdown(bool success);

    virtual void addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw);
    virtual void heartbeatCB(const ros::TimerEvent& event);

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
