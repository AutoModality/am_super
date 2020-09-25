#include <am_super/super_node_mediator.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/adaptor/map.hpp>

namespace am
{
/**
 * The state of the system as the supervisor sees it.*/

SuperNodeMediator::SuperNodeMediator(const std::string& node_name): SUPER_NODE_NAME(node_name)
{
}

/**Encapsulates properties and methods that relate to the transition of states
 * from various sources (SuperState, NodeLifecycle, Flight Controller) to ensure
 * the system state is correct.
 */
struct StateTransition
{
  StateTransition(SuperState _to_state, std::function<bool(SuperNodeMediator::Supervisor&,SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> _check,
                  LifeCycleCommand _life_cycle_command = (LifeCycleCommand)-1)
  {
    to_state = _to_state;
    check = _check;
    life_cycle_command = _life_cycle_command;
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

  bool hasLifecycleCommand()
  {
    //-1 is also the constructor default
    return life_cycle_command != (LifeCycleCommand)-1;
  }
};

/** keyed by the current system state, if the check method passes then the new state will be the given.*/
const std::map<SuperState, StateTransition> state_transitions_ = {
  { SuperState::BOOTING, { SuperState::READY, SuperNodeMediator::checkReadyToArm, LifeCycleCommand::CONFIGURE } },
  { SuperState::READY,
    { SuperState::ARMING, SuperNodeMediator::checkOperatorSignaledToArm } /* no lifecycle command since waiting on operator */ }, 
  { SuperState::ARMING,
    { SuperState::ARMED, SuperNodeMediator::checkArmed, LifeCycleCommand::ACTIVATE } },
  { SuperState::ARMED,
    { SuperState::AUTO, SuperNodeMediator::checkOperatorSignaledToLaunch } },
  { SuperState::AUTO,
    { SuperState::DISARMING, SuperNodeMediator::checkSessionCompleted } },
  { SuperState::DISARMING,
    { SuperState::READY, SuperNodeMediator::checkReadyToArm, LifeCycleCommand::DEACTIVATE }  },
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

bool SuperNodeMediator::nodeNameIsSuper(std::string node_name, SuperNodeMediator& node_mediator)
{
  return SuperNodeMediator::nodeNameStripped(node_name) == node_mediator.getNodeName(); 
}

std::string_view SuperNodeMediator::getNodeName()
{
  return SUPER_NODE_NAME;
}

void  SuperNodeMediator::addSuperToManifest(SuperNodeMediator::Supervisor& supervisor)
{
  supervisor.manifest.push_back(SUPER_NODE_NAME);
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
  // required default state is junk and should not be consulted since not ready
  TransitionInstructions transition_instructions;
  transition_instructions.ready_for_transition = false;
  transition_instructions.resend_life_cycle_command = false;

  // only check those states registered with state_transitions
  if (state_transitions_.count(supervisor.system_state))
  {
    StateTransition transition = state_transitions_.at(supervisor.system_state);

    // each state has a check method providing the logic that should cause transition (based on manifest nodes
    // lifecycle)
    // some transitions happen only when check fails (mostly to abort)
    pair<bool,map<string,string>> check_results = allManifestedNodesCheck(supervisor, *this, transition.check);

    if (check_results.first)
    {
      transition_instructions.ready_for_transition = true;
      transition_instructions.new_state = transition.to_state;
    }
    else
    {
      
      vector<string> failed_nodes;
      boost::copy(check_results.second | boost::adaptors::map_keys, std::back_inserter(failed_nodes));
      transition_instructions.failed_nodes = failed_nodes;
      
      // no transition based on state alone.
      // maybe set the state by the filght controller
      if (transition.flt_ctrl_state_map.count(supervisor.flt_ctrl_state))
      {
        SuperState new_state = transition.flt_ctrl_state_map.at(supervisor.flt_ctrl_state);
        transition_instructions.ready_for_transition = true;
        transition_instructions.new_state = new_state;
      }

      // some check failures send lifecycle commands to encourage nodes to progress so the state can change
      if (transition.hasLifecycleCommand())
      {
        transition_instructions.resend_life_cycle_command = true;
        transition_instructions.life_cycle_command = transition.life_cycle_command;
      }
    }
  }
  return transition_instructions;
}

/** @brief temporary hack to allow manifested nodes to not halt transitions.*/
bool lifeCycleNotYetImplemented(string node_name)
{
  string stripped = SuperNodeMediator::nodeNameStripped(node_name);
  return  
    stripped == "flight_controller" || 
    stripped == "locator" ||
    stripped == "dji_sdk" ||
    stripped == "can_node";
}

bool SuperNodeMediator::checkReadyToArm(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mdediator)
{
  //FIXME: am_super check must be delegated to a single method to do the name check
  return  nr.state == LifeCycleState::INACTIVE || (nr.state == LifeCycleState::ACTIVE && SuperNodeMediator::nodeNameIsSuper(nr.name, node_mdediator));
}

bool SuperNodeMediator::checkOperatorSignaledToArm(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mdediator)
{
  return supervisor.last_op_command_received == OperatorCommand::ARM;
}

bool SuperNodeMediator::checkArmed(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mdediator)
{
  return nr.state == LifeCycleState::ACTIVE;
}

bool SuperNodeMediator::checkOperatorSignaledToLaunch(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mdediator)
{
  return supervisor.last_op_command_received == OperatorCommand::LAUNCH;
}

bool SuperNodeMediator::checkSessionCompleted(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mdediator)
{
  return supervisor.session_completed;
}

pair<bool, map<string, string>> SuperNodeMediator::allManifestedNodesCheck(
    Supervisor& supervisor, SuperNodeMediator& node_mediator, std::function<bool(SuperNodeMediator::Supervisor&,SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check)
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
        success = false;
      }
      else if (lifeCycleNotYetImplemented(node.name))
      {
        error_message = "[WCK2] check skipped: node LifeCycle not yet implemented: " + node.name;
        //not a failure to allow temporary transition 
      }
      else if (!check(supervisor,node,node_mediator))
      {
        //FIXME: this should be a private member in super_node_mediator
        AMLifeCycleMediator life_cycle_mediator; 
        string_view node_state = life_cycle_mediator.stateToString(node.state);
        error_message = "[2OQ0] check failed: node in wrong state " + node.name + ": " + string(node_state);
        success = false;
      }
      else if (node.status == LifeCycleStatus::ERROR)
      {
        error_message = "[AA0A] check failed: node status is ERROR: " + node.name;
        success = false;
      }
    }
    else
    {
      error_message = "[BJIL] check skipped: not manifested: " + node.name;
    }
    if (!error_message.empty())
    {
      failed_nodes.insert(pair<string, string>(node.name, error_message));
    }
  }    // for each node
  return pair(success, failed_nodes);
}

void SuperNodeMediator::parseManifest(Supervisor& supervisor, string manifest)
{
  boost::erase_all(manifest, " ");
  boost::replace_all(manifest,",,",",");
  // if a manifest has been specified
  if (manifest.size())
  {
    // split it based upon commas
    boost::trim_if(manifest,boost::is_any_of(","));
    boost::split(supervisor.manifest, manifest, boost::is_any_of(","));
  }
}

int SuperNodeMediator::nodesOnlineCount(Supervisor supervisor)
{
  return std::count_if(supervisor.nodes.begin(), supervisor.nodes.end(),
                       [](pair<string, SuperNodeInfo> node_entry) { return node_entry.second.online; });
}

int SuperNodeMediator::manifestedNodesOnlineCount(Supervisor supervisor)
{
  return std::count_if(supervisor.nodes.begin(), supervisor.nodes.end(), [](pair<string, SuperNodeInfo> node_entry) {
    return node_entry.second.online && node_entry.second.manifested;
  });
}

map<string,SuperNodeMediator::SuperNodeInfo> SuperNodeMediator::manifestedNodesNotOnline(Supervisor supervisor)
{
  map<string,SuperNodeMediator::SuperNodeInfo> nodes_not_online;
  std::copy_if(supervisor.nodes.begin(), supervisor.nodes.end(), std::inserter(nodes_not_online,nodes_not_online.end()), [](pair<string, SuperNodeInfo> node_entry) {
    return !node_entry.second.online && node_entry.second.manifested;
  });
  return nodes_not_online;
 }

string SuperNodeMediator::manifestedNodesNotOnlineNamesList(Supervisor supervisor)
{
    map<string,SuperNodeMediator::SuperNodeInfo> nodes_not_online=manifestedNodesNotOnline(supervisor);
    vector<string> node_names;
    boost::copy(nodes_not_online | boost::adaptors::map_keys, std::back_inserter(node_names));
    return boost::algorithm::join(node_names, ", ");
}

void SuperNodeMediator::setControllerState(SuperNodeMediator::Supervisor& supervisor, const ControllerState& controller_state)
{
  switch(controller_state)
    {
      case ControllerState::COMPLETED:
        supervisor.session_completed = true;
        break;
    }
}

void SuperNodeMediator::setOperatorCommand(SuperNodeMediator::Supervisor& supevisor, const OperatorCommand& command)
{
  supevisor.last_op_command_received = command;
}

}