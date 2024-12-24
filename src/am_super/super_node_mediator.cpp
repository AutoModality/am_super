#include <am_super/super_node_mediator.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/adaptor/map.hpp>

namespace am
{
/**
 * The state of the system as the supervisor sees it.*/


std::string printLifeCycleState(LifeCycleState lcstate )
{
  switch (lcstate)
  {
    case LifeCycleState::INVALID:
      return "INVALID";
    case LifeCycleState::UNCONFIGURED:
      return "UNCONFIGURED";
    case LifeCycleState::INACTIVE:
      return "INACTIVE";
    case LifeCycleState::ACTIVE:
      return "ACTIVE";
    case LifeCycleState::FINALIZED:
      return "FINALIZED";
    case LifeCycleState::CONFIGURING:
      return "CONFIGURING";
    case LifeCycleState::CLEANING_UP:
      return "CLEANING_UP";
    case LifeCycleState::SHUTTING_DOWN:
      return "SHUTTING_DOWN";
    case LifeCycleState::ACTIVATING:
      return "ACTIVATING";
    case LifeCycleState::DEACTIVATING:
      return "DEACTIVATING";
    case LifeCycleState::ERROR_PROCESSING:
      return "ERROR_PROCESSING";
  }
  return "Unclear LifeCycleState.";
}


std::string printLifeCycleStatus(LifeCycleStatus lcstatus )
{
  switch (lcstatus)
  {
    case LifeCycleStatus::OK:
      return "OK";
    case LifeCycleStatus::WARN:
      return "WARN";
    case LifeCycleStatus::ERROR:
      return "ERROR";
  }
  return "Unclear LifeCycleStatus.";
}

std::string printOperatorCommand(OperatorCommand opcmd )
{
  switch (opcmd)
  {
    case OperatorCommand::ARM:
      return "ARM";
    case OperatorCommand::CANCEL:
      return "CANCEL";
    case OperatorCommand::LAUNCH:
      return "LAUNCH";
    case OperatorCommand::PAUSE:
      return "PAUSE";
    case OperatorCommand::RESUME:
      return "RESUME";
    case OperatorCommand::MANUAL:
      return "MANUAL";
    case OperatorCommand::LANDED:
      return "LANDED";
    case OperatorCommand::ABORT:
      return "ABORT";
    case OperatorCommand::SHUTDOWN:
      return "SHUTDOWN"; 
  }
  return "Unclear OperatorCommand.";
}




SuperNodeMediator::SuperNodeMediator(rclcpp::Node::SharedPtr node, const std::string& node_name):
  SUPER_NODE_NAME(node_name), node_(node),
  state_transitions_({
    // const std::map<SuperState, std::map<SuperState, StateTransition>> state_transitions_ ;
    // StateTransition(SuperState _to_state = NO_SUPER_STATE, std::function<bool(SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> _check = NULL,
    //                 LifeCycleCommand _life_cycle_command = NO_LIFECYCLE_COMMAND, OperatorCommand _operator_command = NO_OPERATOR_COMMAND, 
    //                 ControllerState _controller_state = NO_CONTROLLER_STATE)

    { SuperState::BOOTING, { // from state
      {SuperState::READY, {SuperState::READY, SuperNodeMediator::checkReadyToArm, LifeCycleCommand::CONFIGURE}} // to state
    }},

    
    // FROM HARDIK SIDE:
  
    { SuperState::READY, { // from state
      {SuperState::AUTO, {SuperState::AUTO, SuperNodeMediator::checkArmed, LifeCycleCommand::ACTIVATE, OperatorCommand::LAUNCH}}
      // {SuperState::BOOTING, {SuperState::BOOTING, SuperNodeMediator::checkSuperError}},
    }},

    { SuperState::AUTO, {
      // {SuperState::DISARMING, {SuperState::DISARMING, SuperNodeMediator::checkArmed, StateTransition::NO_LIFECYCLE_COMMAND, StateTransition::NO_OPERATOR_COMMAND, ControllerState::COMPLETED}},
      // {SuperState::READY, {SuperState::READY, SuperNodeMediator::checkArmed, LifeCycleCommand::DEACTIVATE, OperatorCommand::CANCEL}}
      // There is also always the error transition of going back to BOOTING
      // {SuperState::DISARMING, {SuperState::DISARMING, SuperNodeMediator::checkArmed, StateTransition::NO_LIFECYCLE_COMMAND, StateTransition::NO_OPERATOR_COMMAND, ControllerState::COMPLETED}},

      {SuperState::READY, {SuperState::READY, SuperNodeMediator::checkReadyToArm, LifeCycleCommand::DEACTIVATE, OperatorCommand::CANCEL}}
    }},

    // { SuperState::DISARMING, {
      // {SuperState::READY, {SuperState::READY, SuperNodeMediator::checkReadyToArm, LifeCycleCommand::DEACTIVATE}}
    // }},








    // { SuperState:}


    // FROM BEFORE SIDE:

  // { SuperState::READY, {
  //   {SuperState::ARMING, {SuperState::ARMING, SuperNodeMediator::checkReadyToArm, StateTransition::
  //   , OperatorCommand::ARM}},
  //   {SuperState::SHUTDOWN, {SuperState::SHUTDOWN, SuperNodeMediator::checkNodesShuttingDownOrFinalized, LifeCycleCommand::SHUTDOWN, OperatorCommand::SHUTDOWN}}
  
  // }},
  //   { SuperState::ARMING, {
  //     {SuperState::ARMED, {SuperState::ARMED, SuperNodeMediator::checkArmed, LifeCycleCommand::ACTIVATE}}
  //   }},
  // 
  // { SuperState::ARMED, {
  //   {SuperState::AUTO, {SuperState::AUTO, SuperNodeMediator::checkArmed, StateTransition::NO_LIFECYCLE_COMMAND, OperatorCommand::LAUNCH}},
  //   {SuperState::DISARMING, {SuperState::DISARMING, SuperNodeMediator::checkArmed, StateTransition::NO_LIFECYCLE_COMMAND, OperatorCommand::CANCEL}}  
  // }},
  // 
    // { SuperState::AUTO, {
    //   {SuperState::DISARMING, {SuperState::DISARMING, SuperNodeMediator::checkArmed, StateTransition::NO_LIFECYCLE_COMMAND, StateTransition::NO_OPERATOR_COMMAND, ControllerState::COMPLETED}},
    //   {SuperState::MANUAL, {SuperState::MANUAL, SuperNodeMediator::checkReadyToArm, LifeCycleCommand::DEACTIVATE, OperatorCommand::MANUAL}},
    //   {SuperState::SEMI_AUTO, {SuperState::SEMI_AUTO, SuperNodeMediator::checkArmed, StateTransition::NO_LIFECYCLE_COMMAND, OperatorCommand::PAUSE}},
    //   {SuperState::ABORT, {SuperState::ABORT, SuperNodeMediator::checkArmed, StateTransition::NO_LIFECYCLE_COMMAND, OperatorCommand::ABORT}}
    // }},
  // 
  //   { SuperState::SEMI_AUTO, {
  //     {SuperState::AUTO, {SuperState::AUTO, SuperNodeMediator::checkArmed, StateTransition::NO_LIFECYCLE_COMMAND, OperatorCommand::RESUME}},
  //     {SuperState::MANUAL, {SuperState::MANUAL, SuperNodeMediator::checkReadyToArm, LifeCycleCommand::DEACTIVATE, OperatorCommand::MANUAL}}
  //   }},
  // 
  //   { SuperState::DISARMING, {
  //     {SuperState::READY, {SuperState::READY, SuperNodeMediator::checkReadyToArm, LifeCycleCommand::DEACTIVATE}}
  //   }},
  // 
  //   { SuperState::ABORT, {
  //     {SuperState::MANUAL, {SuperState::MANUAL, SuperNodeMediator::checkReadyToArm, LifeCycleCommand::DEACTIVATE, OperatorCommand::MANUAL}},
  //     {SuperState::DISARMING, {SuperState::DISARMING, SuperNodeMediator::checkArmed, StateTransition::NO_LIFECYCLE_COMMAND, StateTransition::NO_OPERATOR_COMMAND, ControllerState::COMPLETED}},
  //   }},
  // 
  //   {SuperState::MANUAL, {
  //     {SuperState::DISARMING, {SuperState::DISARMING, SuperNodeMediator::checkNodesActiveOrInactive, StateTransition::NO_LIFECYCLE_COMMAND, StateTransition::NO_OPERATOR_COMMAND, ControllerState::COMPLETED}},
  //   }}
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
  // supervisor.manifest.insert(supervisor.manifest.begin(), SUPER_NODE_NAME);
}

SuperNodeMediator::SuperNodeInfo SuperNodeMediator::initializeManifestedNode(std::string node_name)
{
  SuperNodeInfo nr;
  nr.name = node_name;
  nr.pid = -1;
  nr.online = false;
  nr.last_contact = node_->now();
  nr.manifested = true;
  nr.state = LifeCycleState::UNCONFIGURED;
  nr.status = LifeCycleStatus::OK;
  return nr;
}

bool SuperNodeMediator::transitionHasOperatorCommand(const StateTransition& transition)
{
  return transition.operator_command != StateTransition::NO_OPERATOR_COMMAND;
}

SuperNodeMediator::StateTransition SuperNodeMediator::getStateTransition(const Supervisor &supervisor)
{ 
  std::map<SuperState, StateTransition> transitions(state_transitions_.at(supervisor.system_state));

  StateTransition attempt_transition;
  ROS_ERROR_STREAM("GetStateTransition...." << printOperatorCommand(supervisor.last_op_command_received));
  for (auto const& [state, transition] : transitions)
  {
    //if this transition has an operator command associated with it and super received it
    if(transitionHasOperatorCommand(transition))
    {
      if(supervisor.last_op_command_received == transition.operator_command)
      {
        ROS_ERROR_STREAM("GetStateTransition: Processing an operator command | " << printOperatorCommand(supervisor.last_op_command_received) << " vs " << printOperatorCommand(transition.operator_command));
        return transition;
      }
    }
    // else if (transition.operator_command == OperatorCommand::LAUNCH && supervisor.start_fp_from_super_)
    // {
    //   // Allows for the mission to be started automatically if the anove parameter is set to true.
    //   ROS_ERROR_STREAM("GetStateTransition: Allowing automatic launch command without operator when blue light is achieved.");
    //   return transition;
    // }
    else if(transitionHasControllerState(transition))
    {
      if(supervisor.last_controller_state_received == transition.controller_state)
      {
        ROS_ERROR_STREAM("GetStateTransition: Processing a controller state transition");
        return transition;
      }
    }
    else
    {
      ROS_ERROR_STREAM("GetStateTransition: Return blank transition");
      return transition;
    }
  }
  ROS_ERROR_STREAM("GetStateTransition: INVALID transition");

  return invalidTransition();
}

SuperNodeMediator::StateTransition SuperNodeMediator::getErrorTransition()
{
  // OLD
  // return {SuperState::SHUTDOWN, SuperNodeMediator::checkNodesShuttingDownOrFinalized, LifeCycleCommand::SHUTDOWN};

  // NEW
  return {SuperState::BOOTING, SuperNodeMediator::checkErrorTransition, LifeCycleCommand::DEACTIVATE}; 
  // It's possible deactivate could be unsuitable if this is an error when you are in READY...?
  // If a node times out, then you need to error out, but the node that comes back online is ACTIVE.... then what.
}

SuperNodeMediator::StateTransition SuperNodeMediator::invalidTransition()
{
  return StateTransition();
}

bool SuperNodeMediator::transitionIsValid(const StateTransition& transition)
{
  return transition.to_state != StateTransition::NO_SUPER_STATE; 
}

SuperNodeMediator::TransitionInstructions SuperNodeMediator::transitionReady(Supervisor& supervisor)
{
  // required default state is junk and should not be consulted since not ready
  TransitionInstructions transition_instructions;
  transition_instructions.ready_for_transition = false;
  transition_instructions.resend_life_cycle_command = false;
  transition_instructions.error_transition = false;

  // Hardik: shortcircuit if we have am_super in error
  // if (supervisor.nodes.at(SUPER_NODE_NAME).status == LifeCycleStatus::ERROR)
  // {
  //   transition_instructions.ready_for_transition = true;
  //   transition_instructions.new_state = getErrorTransition().to_state;
  //   return transition_instructions;
  // }

  // // Come out of error when looking for the next transition if nothing is in error
  // if (supervisor.status_error && supervisor.errored_nodes_.size() == 0)
  // {
  //   ROS_WARN_STREAM("Removing the status_error_ !!");
  //   supervisor.status_error = false;
  // }


      // pair<bool,map<string,pair<bool, string>>> check_results = allManifestedNodesCheck(supervisor, transition.check);
  // Simplified logic that shortcuts getting any other transition.
  ROS_WARN_STREAM("For some reason status_error is true here? : " << supervisor.status_error);
  if (supervisor.status_error)
  {

    if (supervisor.errored_nodes_.size()==0 && allManifestedNodesCheck(supervisor, getErrorTransition().check).first)
    {
      // TransitionInstructions pseudo_transition;
      // pseudo_transition.ready_for_transition = false;
      // pesudo_transition.resend_life_cycle_command = false;
      // pseudo_transition.error_transition = false;
      // pseudo_transition.new_state = getErrorTransition().to_state;
      // if ()
      ROS_WARN_STREAM("Removing the status_error_ !!");
      supervisor.status_error = false;
    }
    else
    {
      // assert(supervisor.errored_nodes_.size() > 0);
      transition_instructions.ready_for_transition = true;
      transition_instructions.new_state = getErrorTransition().to_state;
      transition_instructions.error_transition = true;
      return transition_instructions;
    }
  }
  

  // // bool are_in_error = false;
  // for (pair<string, SuperNodeInfo> nodePair : supervisor.nodes)
  // {
  //   SuperNodeInfo node = nodePair.second;
  //   if (node.status == LifeCycleStatus::ERROR)
  //   {
  //     transition_instructions.ready_for_transition = true;
  //     // are_in_error = true;
  //     supervisor.status_error = true;
  //     transition_instructions.new_state = getErrorTransition().to_state;
  //     transition_instructions.resend_life_cycle_command = true;
  //     // if (supervisor.system_state == SuperState::READY)
  //     // {
  //     //   transition_instructions.life_cycle_command = LifeCycleCommand::CLEANUP;
  //     // }
  //     // else if (supervisor.system_state == SuperState::AUTO)
  //     // {
  //     //   transition_instructions.life_cycle_command = LifeCycleCommand::DEACTIVATE; 
  //     // }
  //     // return transition_instructions;
  //     break;
  //   }
  // }

  // if (are_in_error)
  // {
  //   bool active_true = false;
  //   bool inactive_true = false;

  //   for (pair<string, SuperNodeInfo> nodePair : supervisor.nodes)
  //   {
  //     SuperNodeInfo node = nodePair.second;
  //     // If things are active, make sure they come down inactive
  //     // If things are inactive, then make sure they come down to Unconfingured

  //     if (node.state == LifeCycleState::ACTIVE)
  //     {
  //       active_true = true;
  //     }
  //     if (node.state == LifeCycleState::INACTIVE)
  //     {
  //       inactive_true = true;
  //     }

  //   }

  //   if (active_true)
  //   {
  //     transition_instructions.life_cycle_command = LifeCycleCommand::DEACTIVATE;
  //   }
  //   else if (inactive_true)
  //   {
  //     transition_instructions.life_cycle_command = LifeCycleCommand::CLEANUP;
  //   }
  //   else
  //   {
  //     // This technically should not happen....?
  //     transition_instructions.life_cycle_command = LifeCycleCommand::CONFIGURE;
  //   }
  //   return transition_instructions;
    
  // }



  // only check those states registered with state_transitions
  if (state_transitions_.count(supervisor.system_state))
  { 
    StateTransition transition;

    // TODO: put htis back in when we figure out how errors are going to work
    // if(supervisor.status_error)
    // else
    {
      transition = getStateTransition(supervisor);
    }

    // each state has a check method providing the logic that should cause transition (based on manifest nodes
    // lifecycle)
    // some transitions happen only when check fails (mostly to abort)

    //if there was no statetransition as indicated by the to_state equalling the current state, then don't transition
    if(transitionIsValid(transition))
    {
      pair<bool,map<string,pair<bool, string>>> check_results = allManifestedNodesCheck(supervisor, transition.check);

      //transition to new state if checks passed or forced
      bool checks_passed = check_results.first;
      if (checks_passed || forceTransition(transition.to_state))
      {
        transition_instructions.ready_for_transition = true;
        transition_instructions.new_state = transition.to_state;
      }

      //if checks didn't pass
      if (!checks_passed)
      {
        vector<string> failed_nodes;
        // vector<string> failed_nodes_reasons;
        vector<pair<bool,string>> failed_nodes_reasons;
        boost::copy(check_results.second | boost::adaptors::map_keys, std::back_inserter(failed_nodes));
        boost::copy(check_results.second | boost::adaptors::map_values, std::back_inserter(failed_nodes_reasons));
        transition_instructions.failed_nodes = failed_nodes;
        transition_instructions.failed_nodes_reasons = failed_nodes_reasons;

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


bool SuperNodeMediator::forceTransition(const SuperState& to_state)
{
  return to_state == SuperState::MANUAL || to_state == SuperState::SHUTDOWN;
}

bool SuperNodeMediator::lifeCycleNotYetImplemented(string node_name)
{
  return false;
}

bool SuperNodeMediator::checkReadyToArm(SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{
  ROS_INFO_STREAM("checkReadyToArm: " << nr.name << " is " << printLifeCycleState(nr.state));
  // return  nr.state == LifeCycleState::INACTIVE || (nr.state == LifeCycleState::ACTIVE && node_mediator.nodeNameIsSuper(nr.name));
  return  nr.state == LifeCycleState::INACTIVE && nr.status != LifeCycleStatus::ERROR;
}

bool SuperNodeMediator::checkArmed(SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{
  ROS_INFO_STREAM("checkArmed: " << nr.name << " is " << printLifeCycleState(nr.state));
  return nr.state == LifeCycleState::ACTIVE && nr.status != LifeCycleStatus::ERROR;
}

bool SuperNodeMediator::checkNodesActiveOrInactive(SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{
  return nr.state == LifeCycleState::ACTIVE || nr.state == LifeCycleState::INACTIVE;
}

bool SuperNodeMediator::checkNodesShuttingDownOrFinalized(SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{
  return nr.state == LifeCycleState::SHUTTING_DOWN || nr.state == LifeCycleState::FINALIZED;
}

bool SuperNodeMediator::checkErrorTransition(SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{
  return nr.state == LifeCycleState::INACTIVE;
  // return nr.state != LifeCycleState::ACTIVE;
  // return nr.state == LifeCycleState::UNCONFIGURED | nr.state == LifeCycleState::INACTIVE;
}

// If we use Solution 1, this function should probably be renamed.
bool SuperNodeMediator::checkSuperError(SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator)
{

  // Solution 1:
  // If any node goes into an error or out of inactive/active, then bring the system back to booting
  // return !(nr.state == LifeCycleState::INACTIVE || nr.state == LifeCycleState::ACTIVE) || nr.status == LifeCycleStatus::ERROR;


  // Solution 2: 
  // Ignore all nodes except am_super, and let am_super go into error if it encounters an issue in the manifest. 
  // This requires the AMSuperNodeStats to update.

  // return (node_mediator.nodeNameIsSuper(nr.name) && (nr.state == LifeCycleState::INACTIVE || nr.state == LifeCycleState::ACTIVE) && nr.status == LifeCycleStatus::ERROR );
  // bool normal_node = (!node_mediator.nodeNameIsSuper(nr.name) && (nr.state == LifeCycleState::INACTIVE || nr.state == LifeCycleState::ACTIVE));
  bool normal_node = (!node_mediator.nodeNameIsSuper(nr.name));
  // bool super_node = (node_mediator.nodeNameIsSuper(nr.name) && (nr.state == LifeCycleState::INACTIVE || nr.state == LifeCycleState::ACTIVE) && nr.status == LifeCycleStatus::ERROR );
  bool super_node = (node_mediator.nodeNameIsSuper(nr.name) && (nr.state == LifeCycleState::INACTIVE || nr.state == LifeCycleState::ACTIVE) && nr.status == LifeCycleStatus::ERROR );

  ROS_WARN_STREAM("HARDIK: CHECKSUPERERROR retval:" << (normal_node || super_node)
  << " - name:(" << nr.name << "," << node_mediator.nodeNameIsSuper(nr.name) << ") - state:" << am::printLifeCycleState(nr.state) << " - status:" << printLifeCycleStatus(nr.status));
  // return (normal_node || super_node);


  if (node_mediator.nodeNameIsSuper(nr.name))
  {
    // The node is am_super
    if ((nr.state == LifeCycleState::INACTIVE || nr.state == LifeCycleState::ACTIVE) && nr.status == LifeCycleStatus::ERROR)
    {
      return true; // super is in error
    }
    else
    {
      return false; // super is not in error
    }
  }
  
  // If the node is not am_super, we can ignore it.
  return true;


}


// bool SuperNodeMediator::allManifestedBackToInactiveCheck(Supervisor& supervisor)
// {
//   for (auto nodePair : supervisor.nodes)
//   {
//     SuperNodeInfo node = nodePair.second;

//     if (node.manifested)
//     {

//     }
//   }
// }

// This does the check function on all manifested nodes.
pair<bool, map<string, pair<bool, string>>> SuperNodeMediator::allManifestedNodesCheck(
    Supervisor& supervisor, std::function<bool(SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check)
{
  // The format of this map is: <string node_name, <bool should_I_send_lifecycle_command, string error_message>>
  map<string, pair<bool,string>> failed_nodes;
  bool success = true;
  
  for (pair<string, SuperNodeInfo> nodePair : supervisor.nodes)
  {
    SuperNodeInfo node = nodePair.second;
    std::string error_message;
    bool need_lifecycle_resend;
    // only check manifested nodes, ignore others
    if (node.manifested)
    {
      ROS_WARN_STREAM("hardik-allManifestedNodesCheck: 1");
      if (!node.online)
      {
        ROS_WARN_STREAM("hardik-allManifestedNodesCheck: 1.1");
        error_message = "[U5JB] check failed: node not online: " + node.name;
        success = false;
        need_lifecycle_resend = true;
      }
      else if (lifeCycleNotYetImplemented(node.name))
      {
        ROS_WARN_STREAM("hardik-allManifestedNodesCheck: 1.2");
        error_message = "[WCK2] check skipped: node LifeCycle not yet implemented: " + node.name;
        //not a failure to allow temporary transition until implemented
        need_lifecycle_resend = false;
      }
      else if (!check(node, *this))
      {
        ROS_WARN_STREAM("hardik-allManifestedNodesCheck: 1.3");
        string_view node_state = life_cycle_mediator.stateToString(node.state);
        error_message = "[2OQ0] check failed: node in wrong state " + node.name + ": " + string(node_state);
        ROS_WARN_STREAM(error_message);
        success = false;
        need_lifecycle_resend = true;
      }
      else if (node.status == LifeCycleStatus::ERROR)
      {
        ROS_WARN_STREAM("hardik-allManifestedNodesCheck: 1.4");
        error_message = "[AA0A] check failed: node status is ERROR: " + node.name;
        //if check method passes and we are in error, we want to pass
        // success = false;
        need_lifecycle_resend = false; // TODO from Hardik: Should this be flipped?
      }
      // else if (node.status == LifeCycleState::ACTIVE)
    }
    else
    {
      error_message = "[BJIL] check skipped: not manifested: " + node.name;
      need_lifecycle_resend = false;
    }
    if (!error_message.empty())
    {
      failed_nodes.insert(pair<string, pair<bool,string>>(node.name, pair<bool,string>(need_lifecycle_resend, error_message)));
    }
    ROS_WARN_STREAM("hardik-allManifestedNodesCheck: success:" << success << " latest_node:" << node.name);
  }    // for each node
  return std::pair(success, failed_nodes);
}

// // This does the check function on all manifested nodes.
// pair<bool, map<string, pair<bool, string>>> SuperNodeMediator::allManifestedNodesCheck(
//     Supervisor& supervisor, std::function<bool(SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check)
// {
//   // The format of this map is: <string node_name, <bool should_I_send_lifecycle_command, string error_message>>
//   map<string, pair<bool,string>> failed_nodes;
//   bool success = true;
  
//   for (pair<string, SuperNodeInfo> nodePair : supervisor.nodes)
//   {
//     SuperNodeInfo node = nodePair.second;
//     std::string error_message;
//     bool need_lifecycle_resend;
//     // only check manifested nodes, ignore others
//     if (node.manifested)
//     {
//       ROS_WARN_STREAM("hardik-allManifestedNodesCheck: 1");
//       if (!node.online)
//       {
//         ROS_WARN_STREAM("hardik-allManifestedNodesCheck: 1.1");
//         error_message = "[U5JB] check failed: node not online: " + node.name;
//         // success = false;
//         need_lifecycle_resend = true;
//       }
//       if (lifeCycleNotYetImplemented(node.name))
//       {
//         ROS_WARN_STREAM("hardik-allManifestedNodesCheck: 1.2");
//         error_message = "[WCK2] check skipped: node LifeCycle not yet implemented: " + node.name;
//         //not a failure to allow temporary transition until implemented
//         need_lifecycle_resend = false;
//       }
//       else if (!check(node, *this))
//       {
//         ROS_WARN_STREAM("hardik-allManifestedNodesCheck: 1.3");
//         string_view node_state = life_cycle_mediator.stateToString(node.state);
//         error_message = "[2OQ0] check failed: node in wrong state " + node.name + ": " + string(node_state);
//         success = false;
//         need_lifecycle_resend = true;
//       }
//       else if (node.status == LifeCycleStatus::ERROR && this->nodeNameIsSuper(node.name)) // IN ready, this will catch all errors EXCEPT the am_super one.
//       {
//         ROS_WARN_STREAM("hardik-allManifestedNodesCheck: 1.4");
//         error_message = "[AA0A] check failed: node status is ERROR: " + node.name;
//         //if check method passes and we are in error, we want to pass
//         success = false;
//         need_lifecycle_resend = false; // TODO from Hardik: Should this be flipped?
//       }
//     }
//     else
//     {
//       error_message = "[BJIL] check skipped: not manifested: " + node.name;
//       need_lifecycle_resend = false;
//     }
//     if (!error_message.empty())
//     {
//       failed_nodes.insert(pair<string, pair<bool,string>>(node.name, pair<bool,string>(need_lifecycle_resend, error_message)));
//     }
//     ROS_WARN_STREAM("hardik-allManifestedNodesCheck: success:" << success << " latest_node:" << node.name);

//   }    // for each node
//   return std::pair(success, failed_nodes);
// }


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
  supervisor.last_controller_state_received = controller_state;
}

void SuperNodeMediator::setOperatorCommand(SuperNodeMediator::Supervisor& supevisor, const OperatorCommand& command)
{
  supevisor.last_op_command_received = command;
}

bool SuperNodeMediator::transitionHasLifecycleCommand(const StateTransition& transition)
{
  return transition.life_cycle_command != StateTransition::NO_LIFECYCLE_COMMAND;
}

bool SuperNodeMediator::transitionHasControllerState(const StateTransition& transition) 
{
  return transition.controller_state != StateTransition::NO_CONTROLLER_STATE;
}

void SuperNodeMediator::platformConfigToVariant(const std::string config,
                                                SuperNodeMediator::PlatformVariant &variant)
{
  std::vector<std::string> results;
  boost::split(results, config, [](char c){return c == '_';});
  if (results.size() > 0){
    variant.maker = results.at(0);
  }
  if (results.size() > 1){
    variant.model = results.at(1);
  }
  if (results.size() > 2)
  {
    variant.app = results.at(2);
  }
}

std::string SuperNodeMediator::platformVariantToConfig(const SuperNodeMediator::PlatformVariant &variant)
{
  std::stringstream config;
  const std::string dilimeter = "_"; //must match character in parser method
  if(!variant.maker.empty())
  { 
    config << variant.maker;
    if(!variant.model.empty())
    { 
      config << dilimeter << variant.model;
      if(!variant.app.empty())
      {
        config << dilimeter << variant.app;
      }
    }
  }
  else if(!variant.app.empty())
  {
    std::string any = "*";
    config << any << dilimeter << any << dilimeter << variant.app;
  }
  return config.str();
}

bool SuperNodeMediator::isCorrectPlatform(const SuperNodeMediator::PlatformVariant &required, 
                                          const SuperNodeMediator::PlatformVariant &actual)
{
  //if all is empty, then its a pass
  bool pass = true;

  //maker can be solo, but model must always be with maker
  if(!required.maker.empty())
  {
    pass = pass && required.maker == actual.maker;
    if(!required.model.empty())
    {
      pass = pass && required.model == actual.model;
    }
  }
  //app is optional, but can be by itself
  if(!required.app.empty())
  {
    pass = pass && required.app == actual.app;
  }
  
  return pass;
}

}
