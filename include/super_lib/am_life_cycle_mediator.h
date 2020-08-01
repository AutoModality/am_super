#ifndef AM_LIFE_CYCLE_MEDIATOR_H_
#define AM_LIFE_CYCLE_MEDIATOR_H_

#include <super_lib/am_life_cycle_types.h>

/** Stateless methods providing function without coupling to ROS or any 
 * systems providing testable code.
 */
using namespace am;
class AMLifeCycleMediator
{

public:
    struct LifeCycleInfo
    {
        LifeCycleStatus status;
    };
    
    bool setStatus(const LifeCycleStatus& status, LifeCycleInfo& info);
    LifeCycleStatus getStatus(const LifeCycleInfo& info);
};

#endif