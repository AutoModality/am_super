#ifndef STATE_TRANSITION_H_
#define STATE_TRANSITION_H_ 

#include <am_super/super_node_mediator.h>

namespace am
{
/**Encapsulates properties and methods that relate to the transition of states
 * from various sources (SuperState, NodeLifecycle, Flight Controller) to ensure
 * the system state is correct.
 */
class StateTransition
{

public:
  StateTransition(SuperState _to_state = (SuperState)-1, std::function<bool(SuperNodeMediator::Supervisor&,SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> _check = NULL,
                  LifeCycleCommand _life_cycle_command = (LifeCycleCommand)-1, OperatorCommand _operator_command = (OperatorCommand)-1)
  {
    to_state = _to_state;
    check = _check;
    life_cycle_command = _life_cycle_command;
    operator_command = _operator_command;
    on_check_result = true; //TODO: remove this; we are assuming the check method should always return true now
  }
  /**The future Supervisor.systemState if checks pass.*/
  SuperState to_state;
  /**Function that indicates if the transition is allowed (based on node lifecycle)*/
  std::function<bool(SuperNodeMediator::Supervisor&,SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check;

  /**If the check result matches this value, then transition*/
  bool on_check_result;

  /**State change based on flight controller state 
   * DEPRECATED - Remove when operator_is_ready_to_arm is complete AM-461 
   */
  std::map<SuperNodeMediator::SuperFltCtrlState, SuperState> flt_ctrl_state_map;

  /**Certain states are waiting on nodes to do their thing.  Sending lifecycle commands to new nodes
   * or nodes that missed previous messages will help flush these pending nodes to finish.
   */
  LifeCycleCommand life_cycle_command;

  /** The command super needs to receive in order for us to attempt this transition*/
  OperatorCommand operator_command;

  /**
   * @returns true - operator_command has been assigned 
   */
  bool hasOperatorCommand();

  /**
   * @returns true - life_cycle_command has been assigned
   */
  bool hasLifecycleCommand();

  /**
   * @returns true - to_state has been assigned
   */
  bool isValid();
};

}
#endif