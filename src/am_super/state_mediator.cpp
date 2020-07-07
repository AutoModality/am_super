#include <am_super/state_mediator.h>

namespace am
{

const std::map<SuperState, std::vector<SuperState>> transitions_ =
{
    { SuperState::OFF, {SuperState::BOOTING} },
    { SuperState::BOOTING, {SuperState::READY} },
    { SuperState::READY, {SuperState::ARMING} },
    { SuperState::ARMING, {SuperState::ARMED} },
    { SuperState::ARMED, {SuperState::AUTO, SuperState::ABORT} },
    { SuperState::AUTO, {SuperState::READY,SuperState::SEMI_AUTO,SuperState::HOLD,SuperState::ABORT,SuperState::MANUAL} },
    { SuperState::SEMI_AUTO, {SuperState::AUTO,SuperState::HOLD,SuperState::ABORT,SuperState::MANUAL} },
    { SuperState::HOLD, {SuperState::ABORT,SuperState::MANUAL} },
    { SuperState::ABORT, {SuperState::READY,SuperState::MANUAL} },
    { SuperState::MANUAL, {SuperState::READY} },
    { SuperState::SHUTDOWN, {SuperState::OFF} },
};

StateMediator::StateMediator(){
}

bool StateMediator::allowsTransition(SuperState from, SuperState to)
{
  bool legal = false;
  if(transitions_.count(from) > 0){
    std::vector<SuperState> allowed = transitions_.at(from);
    if(std::find(allowed.begin(),allowed.end(),to) != allowed.end()){
      legal = true;
    }
  }
  return legal;
}

std::vector<SuperState> StateMediator::allSuperStates()
{
  std::vector<SuperState> all;
  for ( int enumIndex = (int) SuperState::OFF; enumIndex <= (int) SuperState::LAST_STATE; enumIndex++ )
  {
    SuperState state = static_cast<SuperState>(enumIndex);
    all.push_back(state);
  }
  return all;
}


};
