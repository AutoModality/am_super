

#ifndef AM_SUPER_INCLUDE_AM_SUPER_SERVICE_H_
#define AM_SUPER_INCLUDE_AM_SUPER_SERVICE_H_

#include <am_super/super_state.h>

namespace am
{

/**
 * Provides all logic about system State allowing the State objects 
 * to be simply data objects.  Allows the ROS node to hold the state of the 
 * system, but delegates all logic to here improving readability, encapsulation 
 * and ability to test.
 * 
 * This class is stateless, but non-static allowing extension to change behavior
 * if different deployments warrant different behavior. 
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
