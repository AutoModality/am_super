#include <am_super/state_mediator.h>

namespace am
{
const std::map<SuperState, std::vector<SuperState>> transitions_ = {
  { SuperState::OFF, { SuperState::BOOTING } },
  { SuperState::BOOTING, { SuperState::READY } },
  { SuperState::READY, { SuperState::ARMING } },
  { SuperState::ARMING, { SuperState::ARMED } },
  { SuperState::ARMED, { SuperState::AUTO, SuperState::ABORT } },
  { SuperState::AUTO,
    { SuperState::READY, SuperState::SEMI_AUTO, SuperState::HOLD, SuperState::ABORT, SuperState::MANUAL } },
  { SuperState::SEMI_AUTO, { SuperState::AUTO, SuperState::HOLD, SuperState::ABORT, SuperState::MANUAL } },
  { SuperState::HOLD, { SuperState::ABORT, SuperState::MANUAL } },
  { SuperState::ABORT, { SuperState::READY, SuperState::MANUAL } },
  { SuperState::MANUAL, { SuperState::READY } },
  { SuperState::SHUTDOWN, { SuperState::OFF } },
};

const std::map<SuperState, std::string_view> state_strings_ = {
  { SuperState::OFF, "OFF" },
  { SuperState::BOOTING, "BOOTING" },
  { SuperState::READY, "READY" },
  { SuperState::ARMING, "ARMING" },
  { SuperState::ARMED, "ARMED" },
  { SuperState::AUTO, "AUTO" },
  { SuperState::SEMI_AUTO, "SEMI_AUTO" },
  { SuperState::HOLD, "HOLD" },
  { SuperState::ABORT, "ABORT" },
  { SuperState::MANUAL, "MANUAL" },
  { SuperState::SHUTDOWN, "SHUTDOWN" },
};

StateMediator::StateMediator()
{
}

bool StateMediator::allowsTransition(SuperState from, SuperState to)
{
  bool legal = false;
  if (transitions_.count(from) > 0)
  {
    std::vector<SuperState> allowed = transitions_.at(from);
    if (std::find(allowed.begin(), allowed.end(), to) != allowed.end())
    {
      legal = true;
    }
  }
  return legal;
}

std::vector<SuperState> StateMediator::allSuperStates()
{
  std::vector<SuperState> all;
  for (int enumIndex = (int)SuperState::OFF; enumIndex <= (int)SuperState::LAST_STATE; enumIndex++)
  {
    SuperState state = static_cast<SuperState>(enumIndex);
    all.push_back(state);
  }
  return all;
}

std::string_view StateMediator::stateToString(SuperState state)
{
  if (state_strings_.count(state) > 0)
  {
    return state_strings_.at(state);
  }
  else
  {
    return INVALID_STRING;
  }
}
};
