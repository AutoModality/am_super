#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_H_
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_H_

#include <string_view>

#include <am_utils/am_ros2_utility.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <super_lib/am_stat_list.h>
#include <super_lib/am_stat_reset.h>
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
class AMLifeCycle : public rclcpp::Node
{
  public:
    static constexpr std::string_view BROADCAST_NODE_NAME = "";
    
    /**Specific parts of the lifecycle where nodes have responsibilities.*/
    LifeCycleState getState() const;

    /**Simple indication of health */
    LifeCycleStatus getStatus() const;
    
    /** @brief string represenation of LifeCycleState*/
    const std::string_view& getStateName();
    
    /** @brief string representation of LifeCycleStatus*/
    const std::string_view& getStatusName();

  private:
    /* Variables to help seperate business logic from AMLifeCycle ROS */
    AMLifeCycleMediator life_cycle_mediator_;
    AMLifeCycleMediator::LifeCycleInfo life_cycle_info_;
    AMLifeCycleMediator::ThrottleInfo throttle_info_;


    /**The moment configuration is requested for this node. Used with 
     * max_configure_seconds_ to allow startup error tolerance.*/
    rclcpp::Time configure_start_time_;

    /** initialization - called by  constructors. 
     */   
    void initialize();

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
    void error(std::string message, std::string error_code, bool forced = false);
    void configureStat(AMStat& stat, std::string name, std::string category="");

  public:
    std::string node_name_;

    /**Maximum time errors will be ignored during configuration. */ 
    int configure_tolerance_s;

    // std::shared_ptr<diagnostic_updater::Updater> updater_;
    diagnostic_updater::Updater updater_;
    AMStatList stats_list_;

    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::Publisher<brain_box_msgs::msg::LifeCycleState>::SharedPtr state_pub_;
    rclcpp::Subscription<brain_box_msgs::msg::LifeCycleCommand>::SharedPtr lifecycle_sub_;

    AMLifeCycle( 
      const std::string & node_name,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    AMLifeCycle(
      const std::string & node_name,
      const std::string & namespace_,
      const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /**
     * @brief Virtual destructor
     */
    virtual ~AMLifeCycle();

    /** Exactly like ros::param, but ROS_INFO's level showing the actual value assigned. 
    */
    template<typename T>
    bool param(const std::string& param_name, T& param_val, const T& default_val);

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
    [[deprecated("use errorTolerant or errorTerminal with message")]]
    void error(std::string error_code="NNLW",bool forced = false);

    /** Reports an error for immediate shutdown without any tolerance. */
    void errorTerminal(std::string message, std::string error_code);

    /** Reports an error, but may not shutdown the system if tolerance is allowed.*/
    void errorTolerant(std::string message, std::string error_code);

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

    /**Initialize statistics by adding to the list*/
    virtual void addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw);


    [[deprecated("configureHzStat with the stat's long name in the yaml")]]
    AMStatReset& configureHzStats(AMStatReset& stats);

    /** Initialize the stats that reset once per second providing the equivalent of rostopic hz to ensure frequency of 
     * publishing.   Allows for overriding values in roslaunch configurations. 
     * Provide the target, which is the approximate value you expect to receive. The warnings and errors will be 
     * provided with tolerance on both sides of the target. 
     * 
     * Configurations key use the stats long name.
     * 
     * setting a target will also set a min/max 5% warn and 10% error
     * no target allows for just min or just max or both.
     * 
     * stats_target_sets_min_max:
     *  hz:
     *   target: 100 # sets min_error=90,min_warn=95,max_warn=105,max_error=110
     * 
     * stats_only_min:
     *  hz:
     *   error:
     *     min: 50
     *   warn:
     *     min: 60
     * 
     * 
     * @param stats to be configured
     * */    
    AMStatReset& configureHzStat(AMStatReset& stats);

    /** Standard configuration of min/max or warn/error for a stat.  
     * 
     * Keyed by the stat long name in the yaml for configuration.
     * 
     * my_stat:
     *   error:
     *     max: 85
     *   warn:
     *     max: 70
     */
    void configureStat(AMStat& stat);

    /** Called periodically by a timer defaulting to 1 second.
     * Useful for checking health regularly, but not during 
     * callbacks which can affect performance and be too granular
     */
    virtual void heartbeatCB();

    void lifecycleCB(const brain_box_msgs::msg::LifeCycleCommand::SharedPtr msg);


    double getThrottleS() const;
    void setThrottleS(const double throttleS);
    double getThrottle();

    /**Providing consistency when logging the current state. */
    void logState();
    
};  // class AMLifeCycle

};  // namespace am

#endif
