

#ifndef AM_SUPER_INCLUDE_AM_SUPER_SERVICE_H_
#define AM_SUPER_INCLUDE_AM_SUPER_SERVICE_H_

#include <am_super/super_state.h>

namespace am
{

/** Stateless class providing logic relating to SuperState rules.
 */
class StateMediator
{
public:
    StateMediator();

    /** 
     * @return true if the new state is acceptable to follow the current.
     */
    bool allowsTransition(SuperState from, SuperState to);
    
private:

};
}

#endif /* AM_SUPER_INCLUDE_AM_SUPER_SERVICE_H_ */
