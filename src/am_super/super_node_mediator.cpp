#include <am_super/super_node_mediator.h>
#include <boost/algorithm/string.hpp>


namespace am
{
/**
 * The state of the system as the supervisor sees it.*/

SuperNodeMediator::SuperNodeMediator()
{
}

/**Encapsulates properties and methods that relate to the transition of states 
 * from various sources (SuperState, NodeLifecycle, Flight Controller) to ensure
 * the system state is correct.
 */
struct StateTransition
{
  StateTransition(SuperState _to_state, std::function<bool(SuperNodeMediator::SuperNodeInfo&)> _check, 
                    bool _on_check_result, std::map<SuperNodeMediator::SuperFltCtrlState,SuperState> _flt_ctrl_state_map,
                    LifeCycleCommand _life_cycle_command = (LifeCycleCommand)-1)
  {
    to_state=_to_state;
    check=_check;
    on_check_result=_on_check_result;
    flt_ctrl_state_map=_flt_ctrl_state_map;
    life_cycle_command = _life_cycle_command;
  }
  /**The future Supervisor.systemState if checks pass.*/
  SuperState to_state;
  /**Function that indicates if the transition is allowed (based on node lifecycle)*/
  std::function<bool(SuperNodeMediator::SuperNodeInfo&)> check;

  /**If the check result matches this value, then transition*/
  bool on_check_result; 

  /**State change based on flight controller state */ 
  std::map<SuperNodeMediator::SuperFltCtrlState,SuperState> flt_ctrl_state_map;

  /**Certain states are waiting on nodes to do their thing.  Sending lifecycle commands to new nodes 
   * or nodes that missed previous messages will help flush these pending nodes to finish.
   */
  LifeCycleCommand life_cycle_command;

  bool hasLifecycleCommand()
  {
    //-1 is also the constructor default
    return life_cycle_command != (LifeCycleCommand) -1;
  }
};

/** keyed by the current system state, if the check method passes then the new state will be the given.*/
const std::map<SuperState, StateTransition> state_transitions_ = {
  { SuperState::BOOTING,   { SuperState::READY,  SuperNodeMediator::checkReadyForConfigureState,true,{} } },
   // TODO: this should wait for operator to arm AM-421
  { SuperState::READY,     { SuperState::ARMING, SuperNodeMediator::checkReadyForActivateState, true,{}, LifeCycleCommand::CONFIGURE } },
  { SuperState::ARMING,    { SuperState::ARMED,  SuperNodeMediator::checkActivateState,         true,{}, LifeCycleCommand::ACTIVATE } },
  { SuperState::ARMED,     { SuperState::ABORT,  SuperNodeMediator::checkActivateState,         false,
    { {SuperNodeMediator::SuperFltCtrlState::AUTO,SuperState::AUTO} } } },
  { SuperState::AUTO,      { SuperState::ABORT,  SuperNodeMediator::checkActivateState,         false,
    { {SuperNodeMediator::SuperFltCtrlState::HOLD,SuperState::SEMI_AUTO} } } },
  { SuperState::SEMI_AUTO, { SuperState::ABORT,  SuperNodeMediator::checkActivateState,         false,
    { {SuperNodeMediator::SuperFltCtrlState::AUTO,SuperState::AUTO} } } },
};


std::string SuperNodeMediator::nodeNameStripped(std::string node_name)
{
  if (node_name.size() > 0 && node_name.at(0) == '/')
  {
    return node_name.substr(1);
  }
  else
  {
    return node_name;
  }
}

SuperNodeMediator::SuperNodeInfo SuperNodeMediator::initializeManifestedNode(std::string node_name)
{
  SuperNodeInfo nr;
  nr.name = node_name;
  nr.pid = -1;
  nr.online = false;
  nr.last_contact = ros::Time(0);
  nr.manifested = true;
  nr.state = LifeCycleState::UNCONFIGURED;
  nr.status = LifeCycleStatus::OK;
  return nr;
}

SuperNodeMediator::TransitionInstructions SuperNodeMediator::transitionReady(Supervisor supervisor)
{
  //required default state is junk and should not be consulted since not ready
  TransitionInstructions transition_instructions;
  transition_instructions.ready_for_transition=false;
  transition_instructions.resend_life_cycle_command=false;

  //only check those states registered with state_transitions
  if(state_transitions_.count(supervisor.system_state))
  {
    StateTransition transition = state_transitions_.at(supervisor.system_state);

    //each state has a check method providing the logic that should cause transition (based on manifest nodes lifecycle)
    //some transitions happen only when check fails (mostly to abort)
    bool check_result = allManifestedNodesCheck(supervisor,transition.check).first;
    if(check_result == transition.on_check_result)
    {
      transition_instructions.ready_for_transition=true;
      transition_instructions.new_state =transition.to_state;
    }
    else
    {
      //no transition based on state alone. 
      //maybe set the state by the filght controller
      if(transition.flt_ctrl_state_map.count(supervisor.flt_ctrl_state))
      {
        SuperState new_state=transition.flt_ctrl_state_map.at(supervisor.flt_ctrl_state);
        transition_instructions.ready_for_transition=true;
        transition_instructions.new_state=new_state;
      }

      //some check failures send lifecycle commands to encourage nodes to progress so the state can change
      if(transition.hasLifecycleCommand())
      {
        transition_instructions.resend_life_cycle_command=true;
        transition_instructions.life_cycle_command=transition.life_cycle_command;
      }
    }
  }
  return transition_instructions;
}

bool SuperNodeMediator::checkReadyForConfigureState(SuperNodeMediator::SuperNodeInfo& nr)
{
  return nr.state == LifeCycleState::UNCONFIGURED || nr.state == LifeCycleState::INACTIVE ||
         nr.state == LifeCycleState::ACTIVE;
}

bool SuperNodeMediator::checkReadyForActivateState(SuperNodeMediator::SuperNodeInfo& nr)
{
  return nr.state == LifeCycleState::INACTIVE || nr.state == LifeCycleState::ACTIVE;
}

bool SuperNodeMediator::checkActivateState(SuperNodeMediator::SuperNodeInfo& nr)
{
  return nr.state == LifeCycleState::ACTIVE;
}

pair<bool, map<string, string>> SuperNodeMediator::allManifestedNodesCheck(
    Supervisor supervisor, std::function<bool(SuperNodeMediator::SuperNodeInfo&)> check)
{
  map<string, string> failed_nodes;

  bool success = true;
  std::string error_message;
  for (pair<string, SuperNodeInfo> nodePair : supervisor.nodes)
  {
    SuperNodeInfo node = nodePair.second;
    // only check manifested nodes, ignore others
    if (node.manifested)
    {
      if (!node.online)
      {
        error_message = "[U5JB] check failed: node not online: " + node.name;
      }
      else if (!check(node))
      {
        string node_state = to_string((int)node.state);  // string(AMLifeCycle::stateToString(node.state));
        error_message = "[2OQ0] check failed: node in wrong state (" + node_state + "): " + node.name;
      }
      else if (node.status == LifeCycleStatus::ERROR)
      {
        error_message = "[AA0A] check failed: node status is ERROR: " + node.name;
      }
      if (!error_message.empty())
      {
        success = false;
        failed_nodes.insert(pair<string, string>(node.name, error_message));
      }
    }  // else not manifested so ignore
  }    // for each node
  return pair(success, failed_nodes);
}

void SuperNodeMediator::parseManifest(Supervisor &supervisor, string manifest)
{
    boost::erase_all(manifest, " ");
    // if a manifest has been specified
    if (manifest.size())
    {
      // split it based upon commas
      boost::split(supervisor.manifest, manifest, boost::is_any_of(","));
    }
}


int SuperNodeMediator::nodesOnlineCount(Supervisor supervisor){
  return std::count_if(supervisor.nodes.begin(), supervisor.nodes.end(), 
    [](pair<string,SuperNodeInfo> node_entry){return node_entry.second.online;});
}

int SuperNodeMediator::manifestedNodesOnlineCount(Supervisor supervisor){
  return std::count_if(supervisor.nodes.begin(), supervisor.nodes.end(), 
    [](pair<string,SuperNodeInfo> node_entry){return node_entry.second.online && node_entry.second.manifested;});
}


}