

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

  /** An indicator that the string is not valid.
   * Instead of throwing an exception which causes difficult to debug seg faults
   * or returning an empty string creating a mystery of what why something is missing.
   **/
  static constexpr std::string_view INVALID_STRING = "INVALID";

  /** See https://automodality.atlassian.net/wiki/spaces/AMROS/pages/929234949/AMROS+System+States
   * @return true if the new state is acceptable to follow the current.
   */
  bool allowsTransition(SuperState from, SuperState to);

  /**
   * @return a vector of all states in order of declaration, excluding LastState which is used for enum iteration
   */
  std::vector<SuperState> allSuperStates();

  /**String representation of the enumeration.
   */
  std::string_view stateToString(SuperState state);

private:
};
}

#endif /* AM_SUPER_INCLUDE_AM_SUPER_SERVICE_H_ */
