#include <super_lib/am_life_cycle_mediator.h>
#include <super_lib/am_life_cycle.h>

namespace am
{

bool AMLifeCycleMediator::setStatus(const LifeCycleStatus& status, LifeCycleInfo& info)
{
    if (status == LifeCycleStatus::LAST_STATUS)
    {
        return false;
    }
    info.status = status;
    return true;
}
LifeCycleStatus AMLifeCycleMediator::getStatus(const LifeCycleInfo& info)
{
    return info.status;
}

} //namespace am