#ifndef AM_LIFE_CYCLE_MEDIATOR_H_
#define AM_LIFE_CYCLE_MEDIATOR_H_

#include <super_lib/am_life_cycle_types.h>

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

public:
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

    static const std::string_view& commandToString(const LifeCycleCommand& command);
    static bool stringToCommand(const std::string& command_str, LifeCycleCommand& command);

    static const std::string_view& statusToString(LifeCycleStatus status);
    static bool stringToStatus(std::string& status_str, LifeCycleStatus& status);

    static const std::string_view& stateToString(LifeCycleState state);
    static bool stringToState(std::string& state_str, LifeCycleState& state);

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