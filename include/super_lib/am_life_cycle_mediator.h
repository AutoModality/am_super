#ifndef AM_LIFE_CYCLE_MEDIATOR_H_
#define AM_LIFE_CYCLE_MEDIATOR_H_

#include <super_lib/am_life_cycle_types.h>
using namespace am;

/** Stateless methods providing function without coupling to ROS or any 
 * systems providing testable code.
 */
class AMLifeCycleMediator
{

public:
    /**
     * Holds information about AMLifeCycle
     */
    struct LifeCycleInfo
    {
        LifeCycleStatus status;
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
    LifeCycleStatus getStatus(const LifeCycleInfo& info);
};

#endif // AM_LIFE_CYCLE_MEDIATOR_H_