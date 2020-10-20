#include <am_super/super_node_mediator.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/adaptor/map.hpp>

namespace am
{
/**
 * The state of the system as the supervisor sees it.*/

SuperNodeMediator::SuperNodeMediator(const std::string& node_name): 
  SUPER_NODE_NAME(node_name),
  state_transitions_({
    { SuperState::BOOTING, {
    {SuperState::READY, {SuperState::READY, SuperNodeMediator::checkReadyToArm, LifeCycleCommand::CONFIGURE}}}},
  { SuperState::READY, {
    {SuperState::ARMING, {SuperState::ARMING, SuperNodeMediator::checkOperatorSignaledToArm }}}},
  {SuperState::ARMING, {
    {SuperState::ARMED, {SuperState::ARMED, SuperNodeMediator::checkArmed, LifeCycleCommand::ACTIVATE}}}},
  {SuperState::ARMED, {
    {SuperState::AUTO, {SuperState::AUTO, SuperNodeMediator::checkOperatorSignaledToLaunch, (LifeCycleCommand)-1, OperatorCommand::LAUNCH}}}},
  {SuperState::AUTO, {
    {SuperState::DISARMING, {SuperState::DISARMING, SuperNodeMediator::checkSessionCompleted}}}},
  {SuperState::DISARMING, {
    {SuperState::READY, {SuperState::READY, SuperNodeMediator::checkReadyToArm, LifeCycleCommand::DEACTIVATE}}}}
})
{

}

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
bool SuperNodeMediator::nodeNameIsSuper(std::string node_name)
{
  return SuperNodeMediator::nodeNameStripped(node_name) == this->getNodeName(); 
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

bool SuperNodeMediator::transitionHasOperatorCommand(const StateTransition& transition)
{
  //-1 is also the constructor default
  return transition.operator_command != (OperatorCommand)-1;
}

SuperNodeMediator::StateTransition SuperNodeMediator::getStateTransition(const Supervisor &supervisor)
{ 
  std::map<SuperState, StateTransition> transitions(state_transitions_.at(supervisor.system_state));

  StateTransition attempt_transition;

  //if there is only one state transition, then attempt this one always
  if(transitions.size() == 1)
  {
    attempt_transition = transitions.begin()->second;
  }

  //FIXME: if more than one, find a state transition to attempt
  else if(transitions.size() > 1)
  {
    for (auto const& [state, transition] : transitions)
    {
      //if this transition has an operator command associated with it and super received it
      if(transitionHasOperatorCommand(transition) && supervisor.last_op_command_received == transition.operator_command)
      {
        attempt_transition = transition;
      }
      //TODO: if this transition has a controller state associated with it and super has received it
    }
  }
  else
  {
    //if no statetransition was found, return a new stateTransition with the to_state equal to the current state. This indicates no transition should occur
    attempt_transition = invalidTransition();
  }
  
  return attempt_transition;
}

SuperNodeMediator::StateTransition SuperNodeMediator::invalidTransition()
{
  return StateTransition();
}

bool SuperNodeMediator::transitionIsValid(const StateTransition& transition)
{
  return transition.to_state != (SuperState)-1; //FIXME: reference constant
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
    StateTransition transition = getStateTransition(supervisor);
    // each state has a check method providing the logic that should cause transition (based on manifest nodes
    // lifecycle)
    // some transitions happen only when check fails (mostly to abort)

    //if there was no statetransition as indicated by the to_state equalling the current state, then don't transition
    if(transitionIsValid(transition))
    {
      pair<bool,map<string,string>> check_results = allManifestedNodesCheck(supervisor, transition.check);

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
        if (transitionHasLifecycleCommand(transition))
        {
          transition_instructions.resend_life_cycle_command = true;
          transition_instructions.life_cycle_command = transition.life_cycle_command;
        }
      }
    }
  }
  return transition_instructions;
}

bool SuperNodeMediator::lifeCycleNotYetImplemented(string node_name)
{
  string stripped = SuperNodeMediator::nodeNameStripped(node_name);
  return  
    stripped == "flight_controller" || 
    stripped == "locator" ||
    stripped == "dji_sdk" ||
    stripped == "can_node";
}

bool SuperNodeMediator::checkReadyToArm(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{
  return  nr.state == LifeCycleState::INACTIVE || (nr.state == LifeCycleState::ACTIVE && node_mediator.nodeNameIsSuper(nr.name));
}

bool SuperNodeMediator::checkOperatorSignaledToArm(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{
  return supervisor.last_op_command_received == OperatorCommand::ARM;
}

bool SuperNodeMediator::checkArmed(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{
  return nr.state == LifeCycleState::ACTIVE;
}

bool SuperNodeMediator::checkOperatorSignaledToLaunch(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{
  return supervisor.last_op_command_received == OperatorCommand::LAUNCH;
}

bool SuperNodeMediator::checkSessionCompleted(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{
  return supervisor.session_completed;
}

bool SuperNodeMediator::checkOperatorSignaledToManual(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{
  return supervisor.last_op_command_received == OperatorCommand::MANUAL;
}

pair<bool, map<string, string>> SuperNodeMediator::allManifestedNodesCheck(
    Supervisor& supervisor, std::function<bool(SuperNodeMediator::Supervisor&,SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check)
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
      else if (!check(supervisor, node, *this))
      {
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

bool SuperNodeMediator::transitionHasLifecycleCommand(const StateTransition& transition)
{
  //-1 is also the constructor default
  return transition.life_cycle_command != (LifeCycleCommand)-1;
}

}