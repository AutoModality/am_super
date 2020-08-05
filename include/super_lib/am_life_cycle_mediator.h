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
    /**
     * @brief Sets the current LifeCycleStatus in the mediator
     * 
     * @param status    status that we want to set
     * @param info      mediator enum holding information about LifeCycle
     * 
     * @returns true    status is valid and mediator info.status was updated
     * @returns false   status is invalid and mediator info.status was not updated
     */ 
    bool setStatus(const LifeCycleStatus& status, LifeCycleInfo& info);

    /**
     * @brief Gets the current LifeCycleStatus in the mediator
     * 
     * @param info mediator enum holding information about LifeCycle
     * 
     * @returns info.state - the current LifeCycleStatus in the mediator
     */ 
    LifeCycleStatus getStatus(const LifeCycleInfo& info) const;

    bool setState(const LifeCycleState& state, LifeCycleInfo& info);
    LifeCycleState getState(const LifeCycleInfo& info) const;

    const std::string_view& commandToString(const LifeCycleCommand& command);
    bool stringToCommand(const std::string& command_str, LifeCycleCommand& command);

    const std::string_view& statusToString(LifeCycleStatus status);
    bool stringToStatus(std::string& status_str, LifeCycleStatus& status);

    const std::string_view& stateToString(LifeCycleState state);
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
};
}
#endif // AM_LIFE_CYCLE_MEDIATOR_H_