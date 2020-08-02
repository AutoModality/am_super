#include <super_lib/am_life_cycle_mediator.h>
#include <super_lib/am_life_cycle.h>

namespace am
{

bool AMLifeCycleMediator::setStatus(const LifeCycleStatus& status, LifeCycleInfo& info)
{
  if (status == LifeCycleStatus::LAST_STATUS) 
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
  if (state == LifeCycleState::LAST_STATE) 
    return false;
    
  info.state = state;
  return true;
}

LifeCycleState AMLifeCycleMediator::getState(const LifeCycleInfo& info)
{
  return info.state;
}


} //namespace am