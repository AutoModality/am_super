#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_H_
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_H_

#include <string_view>

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticStatus.h>

#include <super_lib/am_stat_list.h>
#include <super_lib/am_life_cycle_types.h>

namespace am
{
class AMLifeCycle
{
private:
  static constexpr double DEFAULT_OK_THROTTLE_S = 10.0;
  static constexpr double DEFAULT_WARN_THROTTLE_S = 2.0;
  static constexpr double DEFAULT_ERROR_THROTTLE_S = 1.0;

  LifeCycleState state_;
  LifeCycleStatus status_;
  double ok_throttle_s_ = DEFAULT_OK_THROTTLE_S;
  double warn_throttle_s_ = DEFAULT_WARN_THROTTLE_S;
  double error_throttle_s_ = DEFAULT_ERROR_THROTTLE_S;

  void setState(const LifeCycleState state);

  void transition(std::string transition_name, LifeCycleState initial_state, LifeCycleState transition_state,
                  LifeCycleState new_state, std::function<void(void)> on_function);
  void doTransition(std::string transition_name, bool success, LifeCycleState success_state,
                    LifeCycleState failure_state);

public:
  static constexpr std::string_view BROADCAST_NODE_NAME = "";

  /* State string messages for LifeCycle */
  static constexpr std::string_view STATE_INVALID_STRING = "INVALID";
  static constexpr std::string_view STATE_UNCONFIGURED_STRING = "UNCONFIGURED";
  static constexpr std::string_view STATE_INACTIVE_STRING = "INACTIVE";
  static constexpr std::string_view STATE_ACTIVE_STRING = "ACTIVE";
  static constexpr std::string_view STATE_FINALIZED_STRING = "FINALIZED";
  static constexpr std::string_view STATE_CONFIGURING_STRING = "CONFIGURING";
  static constexpr std::string_view STATE_CLEANING_UP_STRING = "CLEANING_UP";
  static constexpr std::string_view STATE_ACTIVATING_STRING = "ACTIVATING";
  static constexpr std::string_view STATE_DEACTIVATING_STRING = "DEACTIVATING";
  static constexpr std::string_view STATE_ERROR_PROCESSING_STRING = "ERROR_PROCESSING";
  static constexpr std::string_view STATE_SHUTTING_DOWN = "SHUTTING_DOWN";  

  static constexpr std::string_view STATUS_OK_STRING = "OK";
  static constexpr std::string_view STATUS_WARN_STRING = "WARN";
  static constexpr std::string_view STATUS_ERROR_STRING = "ERROR";

  static constexpr std::string_view COMMAND_CREATE_STRING = "CREATE";
  static constexpr std::string_view COMMAND_CONFIGURE_STRING = "CONFIGURE";
  static constexpr std::string_view COMMAND_CLEANUP_STRING = "CLEANUP";
  static constexpr std::string_view COMMAND_ACTIVATE_STRING = "ACTIVATE";
  static constexpr std::string_view COMMAND_DEACTIVATE_STRING = "DEACTIVATE";
  static constexpr std::string_view COMMAND_SHUTDOWN_STRING = "SHUTDOWN";
  static constexpr std::string_view COMMAND_DESTROY_STRING = "DESTROY";

  static constexpr std::string_view EMPTY_STRING = "";

  /**
   * @brief Stores all states for LifeCycleState into a vector
   * @returns vector of LifeCycleStates
   */
  static const std::vector<LifeCycleState> getLifeCycleStates();

  /**
   * @brief Stores all commands for LifeCycleCommand into a vector
   * @returns vector of LifeCycleCommands
   */
  static const std::vector<LifeCycleCommand> getLifeCycleCommands();

  /**
   * @brief Converts a LifeCycleState into the proper string representation. 
   * If the LifeCycleState is not a valid one, returns "INVALID"
   * 
   * @param state LifeCycleState enum representing the state of LifeCycle
   * 
   * @returns The string that represents the state. "INVALID" if invalid.
   */ 
  static const std::string_view& stateToString(LifeCycleState state);

  /**
   * @brief Reads the string passed in and stores into 'state' the respective
   * LifeCycleState. If the string is not a valid one, the 'state' passed in 
   * is unchanged
   * 
   * @param state_str the string that is converted to a state and stored in 'state'
   * @param state holds the current state
   * 
   * @returns true if the state_str is valid and state was updated
   * @returns false if the state_str is invalid and state was unchanged
   */ 
  static bool stringToState(std::string& state_str, LifeCycleState& state);

  
  static const std::string_view& statusToString(LifeCycleStatus status);
  static bool stringToStatus(std::string& status_str, LifeCycleStatus& status);

  /**
   * @brief Converts a LifeCycleCommand into its proper string representation. 
   * If the LifeCycleCommand is not a valid one, returns "INVALID"
   * 
   * @param command LifeCycleCommand enum representing the command
   * 
   * @returns The string that represents the command. "INVALID" if invalid.
   */ 
  static const std::string_view& commandToString(LifeCycleCommand command);

  /**
   * @brief Reads the string passed in and stores into 'command' the respective
   * LifeCycleCommand. If the string is not a valid one, the 'command' passed in 
   * is unchanged
   * 
   * @param command_str the string that is converted into a command and stored in 'command'
   * @param command holds the current command
   * 
   * @returns true if the command_str is valid and state was updated
   * @returns false if the command_str is invalid and state was unchanged
   */ 
  static bool stringToCommand(std::string& command_str, LifeCycleCommand& command);

protected:
  std::string node_name_;

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

  LifeCycleState getState() const;
  LifeCycleStatus getStatus() const;
  void setStatus(const LifeCycleStatus status);
  double getThrottleS() const;
  void setThrottleS(const double throttleS);

  /**Providing consistency when logging the current state. */
  void logState();
  
};  // class AMLifeCycle

};  // namespace am

#endif
