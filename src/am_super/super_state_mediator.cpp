#include <am_super/super_state_mediator.h>

namespace am
{
/**Local data class providing capabilities for states.
 * This approach encapsulates features available for SuperState avoiding
 * issues with switch statements that can miss required functionality.
 */
struct SuperStateInfo
{
  std::string_view name_;
  std::vector<SuperState> allowed_transitions_;
  SuperStateInfo(std::string_view name, std::vector<SuperState> allowed_transitions)
  {
    name_ = name;
    allowed_transitions_ = allowed_transitions;
    if (allowed_transitions.size() == 0)
    {
      throw std::runtime_error("All states must have a transition");
    }
    if (name.empty())
    {
      throw std::runtime_error("Name is required");
    }
  }
};

/**Registered constants for states mapping to values. */
const std::map<SuperState, SuperStateInfo> state_info_ = {
  { SuperState::OFF, { "OFF", { SuperState::BOOTING } } },
  { SuperState::BOOTING, { "BOOTING", { SuperState::READY } } },
  { SuperState::READY, { "READY", { SuperState::ARMING } } },
  { SuperState::ARMING, { "ARMING", { SuperState::ARMED } } },
  { SuperState::ARMED, { "ARMED", { SuperState::AUTO, SuperState::ABORT } } },
  { SuperState::AUTO,
    { "AUTO", { SuperState::READY, SuperState::SEMI_AUTO, SuperState::HOLD, SuperState::ABORT, SuperState::MANUAL } } },
  { SuperState::SEMI_AUTO,
    { "SEMI_AUTO", { SuperState::AUTO, SuperState::HOLD, SuperState::ABORT, SuperState::MANUAL } } },
  { SuperState::HOLD, { "HOLD", { SuperState::ABORT, SuperState::MANUAL } } },
  { SuperState::ABORT, { "ABORT", { SuperState::READY, SuperState::MANUAL } } },
  { SuperState::MANUAL, { "MANUAL", { SuperState::READY } } },
  { SuperState::SHUTDOWN, { "SHUTDOWN", { SuperState::OFF } } },
};

SuperStateMediator::SuperStateMediator()
{
}

SuperStateInfo info(SuperState state)
{
  if (state_info_.count(state) > 0)
  {
    return state_info_.at(state);
  }
  else
  {
    throw std::runtime_error("State information not registered");
  }
}
bool SuperStateMediator::allowsTransition(SuperState from, SuperState to)
{
  bool legal = false;

  std::vector<SuperState> allowed = info(from).allowed_transitions_;
  if (std::find(allowed.begin(), allowed.end(), to) != allowed.end())
  {
    legal = true;
  }
  return legal;
}

std::vector<SuperState> SuperStateMediator::allSuperStates()
{
  std::vector<SuperState> all;
  for (int enumIndex = (int)FIRST_SUPER_STATE; enumIndex <= (int)LAST_SUPER_STATE; enumIndex++)
  {
    SuperState state = static_cast<SuperState>(enumIndex);
    all.push_back(state);
  }
  return all;
}

std::string_view SuperStateMediator::stateToString(SuperState state)
{
  return info(state).name_;
};
}