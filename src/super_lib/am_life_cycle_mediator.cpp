#include <super_lib/am_life_cycle_mediator.h>
#include <super_lib/am_life_cycle.h>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>



namespace am
{

typedef boost::bimap<std::string_view, am::LifeCycleCommand> str_command_bimap;
const str_command_bimap str_command_bimap_ = boost::assign::list_of< str_command_bimap::relation > 
  (AMLifeCycle::COMMAND_ACTIVATE_STRING, LifeCycleCommand::ACTIVATE)
  (AMLifeCycle::COMMAND_CLEANUP_STRING, LifeCycleCommand::CLEANUP)
  (AMLifeCycle::COMMAND_CONFIGURE_STRING, LifeCycleCommand::CONFIGURE)
  (AMLifeCycle::COMMAND_CREATE_STRING, LifeCycleCommand::CREATE)
  (AMLifeCycle::COMMAND_DEACTIVATE_STRING, LifeCycleCommand::DEACTIVATE)
  (AMLifeCycle::COMMAND_DESTROY_STRING, LifeCycleCommand::DESTROY)
  (AMLifeCycle::COMMAND_SHUTDOWN_STRING, LifeCycleCommand::SHUTDOWN);

const std::string_view& AMLifeCycleMediator::commandToString(const LifeCycleCommand& command)
{
  if(str_command_bimap_.right.count(command))
    {
      return str_command_bimap_.right.at(command);
    }
  return AMLifeCycle::EMPTY_STRING;
}

bool AMLifeCycleMediator::stringToCommand(const std::string& command_str, LifeCycleCommand& command)
{
  if(str_command_bimap_.left.count(command_str))
  {
    command = str_command_bimap_.left.at(command_str);
    return true;
  }
  return false;
}

typedef boost::bimap<std::string_view, am::LifeCycleStatus> str_status_bimap;
const str_status_bimap str_status_bimap_ = boost::assign::list_of< str_status_bimap::relation > 
  (AMLifeCycle::STATUS_OK_STRING, LifeCycleStatus::OK)
  (AMLifeCycle::STATUS_WARN_STRING, LifeCycleStatus::WARN)
  (AMLifeCycle::STATUS_ERROR_STRING, LifeCycleStatus::ERROR);

const std::string_view& AMLifeCycleMediator::statusToString(LifeCycleStatus status)
{
  if(str_status_bimap_.right.count(status))
    {
      return str_status_bimap_.right.at(status);
    }
  return AMLifeCycle::EMPTY_STRING;
}

bool AMLifeCycleMediator::stringToStatus(std::string& status_str, LifeCycleStatus& status)
{
  if(str_status_bimap_.left.count(status_str))
  {
    status = str_status_bimap_.left.at(status_str);
    return true;
  }
  return false;
}

typedef boost::bimap<std::string_view, am::LifeCycleState> str_state_bimap;
const str_state_bimap str_state_bimap_ = boost::assign::list_of< str_state_bimap::relation > 
  (AMLifeCycle::STATE_INVALID_STRING, LifeCycleState::INVALID)
  (AMLifeCycle::STATE_UNCONFIGURED_STRING, LifeCycleState::UNCONFIGURED)
  (AMLifeCycle::STATE_INACTIVE_STRING, LifeCycleState::INACTIVE)
  (AMLifeCycle::STATE_ACTIVE_STRING, LifeCycleState::ACTIVE)
  (AMLifeCycle::STATE_FINALIZED_STRING, LifeCycleState::FINALIZED)
  (AMLifeCycle::STATE_CONFIGURING_STRING, LifeCycleState::CONFIGURING)
  (AMLifeCycle::STATE_CLEANING_UP_STRING, LifeCycleState::CLEANING_UP)
  (AMLifeCycle::STATE_ACTIVATING_STRING, LifeCycleState::ACTIVATING)
  (AMLifeCycle::STATE_DEACTIVATING_STRING, LifeCycleState::DEACTIVATING)
  (AMLifeCycle::STATE_ERROR_PROCESSING_STRING, LifeCycleState::ERROR_PROCESSING)
  (AMLifeCycle::STATE_SHUTTING_DOWN, LifeCycleState::SHUTTING_DOWN);

const std::string_view& AMLifeCycleMediator::stateToString(LifeCycleState state)
{
    if(str_state_bimap_.right.count(state))
    {
      return str_state_bimap_.right.at(state);
    }
    return AMLifeCycle::STATE_INVALID_STRING;
}

bool AMLifeCycleMediator::stringToState(std::string& state_str, LifeCycleState& state)
{
  if(str_state_bimap_.left.count(state_str))
  {
    state = str_state_bimap_.left.at(state_str);
    return true;
  }
  return false;
}

bool AMLifeCycleMediator::setStatus(const LifeCycleStatus& status, LifeCycleInfo& info)
{
  if (status < AMLifeCycleMediator::FIRST_STATUS || status > AMLifeCycleMediator::LAST_STATUS) 
    return false;

  info.status = status;
  return true;
}
LifeCycleStatus AMLifeCycleMediator::getStatus(const LifeCycleInfo& info)
{
  return info.status;
}

bool AMLifeCycleMediator::setState(const LifeCycleState& state, LifeCycleInfo& info)
{
  if (state < AMLifeCycleMediator::FIRST_STATE || state > AMLifeCycleMediator::LAST_STATE) 
    return false;
    
  info.state = state;
  return true;
}

LifeCycleState AMLifeCycleMediator::getState(const LifeCycleInfo& info)
{
  return info.state;
}

const std::vector<LifeCycleCommand> AMLifeCycleMediator::getLifeCycleCommands()
{
  std::vector<LifeCycleCommand> all;
  for (int enumIndex = (int)LifeCycleCommand::CREATE; enumIndex <= (int)LifeCycleCommand::LAST_COMMAND; enumIndex++)
  {
    LifeCycleCommand command = static_cast<LifeCycleCommand>(enumIndex);
    all.push_back(command);
  }
  return all;
}

const std::vector<LifeCycleState> AMLifeCycleMediator::getLifeCycleStates()
{
  std::vector<LifeCycleState> all;
  for (int enumIndex = (int)FIRST_STATE; enumIndex <= (int)LAST_STATE; enumIndex++)
  {
    LifeCycleState state = static_cast<LifeCycleState>(enumIndex);
    all.push_back(state);
  }
  return all;
}

const std::vector<LifeCycleStatus> AMLifeCycleMediator::getLifeCycleStatuses()
{
  std::vector<LifeCycleStatus> all;
  for (int enumIndex = (int)FIRST_STATUS; enumIndex <= (int)LAST_STATUS; enumIndex++)
  {
    LifeCycleStatus Status = static_cast<LifeCycleStatus>(enumIndex);
    all.push_back(Status);
  }
  return all;
}

} //namespace am