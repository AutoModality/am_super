#include <am_super/state_mediator.h>

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
StateMediator::StateMediator()
{
}

bool StateMediator::allowsTransition(SuperState from, SuperState to)
{
  bool legal = false;
  switch (from)
  {
    case SuperState::OFF:
      if (to == SuperState::BOOTING)
      {
        legal = true;
      }
      break;
    case SuperState::BOOTING:
      if (to == SuperState::READY)
      {
        legal = true;
      }
      break;
    case SuperState::READY:
      if (to == SuperState::ARMING)
      {
        legal = true;
      }
      else if (to == SuperState::ARMING)
      {
        legal = true;
      }
      break;
    case SuperState::ARMING:
      if (to == SuperState::ARMED)
      {
        legal = true;
      }
      break;
    case SuperState::ARMED:
      // TODO: remove ABORT state here once we know how to deal with arming errors (should go back to READY).
      if (to == SuperState::AUTO || to == SuperState::ABORT)
      {
        legal = true;
      }
      break;
    case SuperState::AUTO:
      if (to == SuperState::READY || to == SuperState::SEMI_AUTO || to == SuperState::HOLD || to == SuperState::ABORT ||
          to == SuperState::MANUAL)
      {
        legal = true;
      }
      break;
    case SuperState::SEMI_AUTO:
      if (to == SuperState::AUTO || to == SuperState::HOLD || to == SuperState::ABORT || to == SuperState::MANUAL)
      {
        legal = true;
      }
      break;
    case SuperState::HOLD:
      if (to == SuperState::ABORT || to == SuperState::MANUAL)
      {
        legal = true;
      }
      break;
    case SuperState::ABORT:
      if (to == SuperState::READY || to == SuperState::MANUAL)
      {
        legal = true;
      }
      break;
    case SuperState::MANUAL:
      if (to == SuperState::READY)
      {
        legal = true;
      }
      break;
    case SuperState::SHUTDOWN:
      if (to == SuperState::OFF)
      {
        legal = true;
      }
      break;
    default:
    {
      throw std::invalid_argument("Unhandled state ");
    }
  }
  return legal;
}
};
