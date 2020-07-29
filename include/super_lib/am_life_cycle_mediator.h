#include <am_life_cycle_types.h>

/** Stateless methods providing function without coupling to ROS or any 
 * systems providing testable code.
 */
class AMLifeCycleMediator
{

public:
    struct LifeCycleInfo
    {
        LifeCycleStatus status;
    }
    
    void setStatus(LifeCycleStatus status, LifeCycleInfo info);
    LifeCycleStatus getLifeCycleStatus(LifeCycleInfo info);
}