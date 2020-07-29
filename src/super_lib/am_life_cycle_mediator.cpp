#include <super_lib/am_life_cycle_mediator.h>


    bool AMLifeCycleMediator::setStatus(LifeCycleStatus status, LifeCycleInfo info)
    {
        if (status_ == LifeCycleStatus::LAST_STATUS)
        {
           return false;
        }
        else
        {
            status_ = status;
            return true;
        }
        
    }
    LifeCycleStatus AMLifeCycleMediator::getLifeCycleStatus(LifeCycleInfo info);

    {
        return info.status;
    }
