#ifndef AM_LIFE_CYCLE_MEDIATOR_H_
#define AM_LIFE_CYCLE_MEDIATOR_H_

#include <super_lib/am_life_cycle_types.h>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>

typedef boost::bimap<std::string_view, am::LifeCycleCommand> str_command_bimap;
typedef boost::bimap<std::string_view, am::LifeCycleStatus> str_status_bimap;
typedef boost::bimap<std::string_view, am::LifeCycleState> str_state_bimap;

namespace am 
{

/** Stateless methods providing function without coupling to ROS or any 
 * systems providing testable code.
 */
class AMLifeCycleMediator
{
  private:
    static const LifeCycleStatus FIRST_STATUS = LifeCycleStatus::OK;
    static const LifeCycleStatus LAST_STATUS = LifeCycleStatus::ERROR;

    static const LifeCycleState FIRST_STATE = LifeCycleState::INVALID;
    static const LifeCycleState LAST_STATE = LifeCycleState::ERROR_PROCESSING;

    static const LifeCycleCommand FIRST_COMMAND = LifeCycleCommand::CREATE;
    static const LifeCycleCommand LAST_COMMAND = LifeCycleCommand::DESTROY;

    str_command_bimap str_command_bimap_;
    str_status_bimap str_status_bimap_;
    str_state_bimap str_state_bimap_;

    /* String messages for mapping */
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

    static constexpr double DEFAULT_OK_THROTTLE_S = 10.0;
    static constexpr double DEFAULT_WARN_THROTTLE_S = 2.0;
    static constexpr double DEFAULT_ERROR_THROTTLE_S = 1.0;

  public:
    AMLifeCycleMediator();
    /**
     * Holds information about AMLifeCycle
     */
    struct LifeCycleInfo
    {
      LifeCycleStatus status;
      LifeCycleState state;
    };
    struct ThrottleInfo
    {
      double ok_throttle_s = DEFAULT_OK_THROTTLE_S;
      double warn_throttle_s = DEFAULT_WARN_THROTTLE_S;
      double error_throttle_s = DEFAULT_ERROR_THROTTLE_S;
    };
    /**
     * @brief Sets the current LifeCycleStatus in LifeCycleInfo
     * 
     * @param status    status that we want to set
     * @param info      mediator enum holding information about LifeCycle
     * 
     * @returns true    status is valid and mediator info.status was updated
     * @returns false   status is invalid and mediator info.status was not updated
     */ 
    bool setStatus(const LifeCycleStatus& status, LifeCycleInfo& info);

    /**
     * @brief Gets the current LifeCycleStatus from LifeCycleInfo
     * 
     * @param info mediator enum holding information about LifeCycle
     * 
     * @returns info.state - the current LifeCycleStatus in the mediator
     */ 
    LifeCycleStatus getStatus(const LifeCycleInfo& info) const;

    /**
     * @brief Sets the current LifeCycleState in LifeCycleInfo
     * 
     * @param state     state that we want to set
     * @param info      mediator enum holding information about LifeCycle
     * 
     * @returns true    state is valid and info.state was updated
     * @returns false   state is invalid and info.state was not updated
     */ 
    bool setState(const LifeCycleState& state, LifeCycleInfo& info);

    /**
     * @brief Gets the current LifeCycleState from LifeCycleInfo
     * 
     * @param info mediator enum holding information about LifeCycle
     * 
     * @returns info.state - the current LifeCycleState in info
     */ 
    LifeCycleState getState(const LifeCycleInfo& info) const;

    /**
     * @brief Converts a LifeCycleCommand into its proper string representation. 
     * If the LifeCycleCommand is not a valid one, returns ""
     * 
     * @param command LifeCycleCommand enum representing the command
     * 
     * @returns The string that represents the command. "" if invalid.
     */ 
    const std::string_view& commandToString(const LifeCycleCommand& command);
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
    bool stringToCommand(const std::string& command_str, LifeCycleCommand& command);

    /**
     * @brief Converts a LifeCycleStatus into its proper string representation. 
     * If the LifeCycleStatus is not a valid one, returns ""
     * 
     * @param status LifeCycleStatus enum representing the status
     * 
     * @returns The string that represents the status. "" if invalid.
     */ 
    const std::string_view& statusToString(LifeCycleStatus status);
    /**
     * @brief Reads the string passed in and stores into 'status' the respective
     * LifeCycleStatus. If the string is not a valid one, the 'status' passed in 
     * is unchanged
     * 
     * @param status_str the string that is converted into a status and stored in 'status'
     * @param status holds the current status
     * 
     * @returns true if the status_str is valid and state was updated
     * @returns false if the status_str is invalid and state was unchanged
     */ 
    bool stringToStatus(std::string& status_str, LifeCycleStatus& status);
    /**
     * @brief Converts a LifeCycleState into the proper string representation. 
     * If the LifeCycleState is not a valid one, returns "INVALID"
     * 
     * @param state LifeCycleState enum representing the state of LifeCycle
     * 
     * @returns The string that represents the state. "INVALID" if invalid.
     */ 
    const std::string_view& stateToString(LifeCycleState state);

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
    bool stringToState(std::string& state_str, LifeCycleState& state);

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
     * @brief Stores all status' for LifeCycleStatus into a vector
     * @returns vector of LifeCycleStatus
     */
    static const std::vector<LifeCycleStatus> getLifeCycleStatuses();

    /**
     * @brief 
     * 
     * @param 
     * 
     * @returns 
     */ 
    void setThrottleS(const double& throttleS, ThrottleInfo& throttle);

    /**
     * @brief 
     * 
     * @param 
     * 
     * @returns 
     */ 
    ThrottleInfo getThrottleDefaults();

};
}
#endif // AM_LIFE_CYCLE_MEDIATOR_H_