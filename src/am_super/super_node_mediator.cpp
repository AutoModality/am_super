#include <am_super/super_node_mediator.h>

namespace am
{

/**
 * The state of the system as the supervisor sees it.*/


SuperNodeMediator::SuperNodeMediator()
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


pair<bool,map<string,string>> SuperNodeMediator::allManifestedNodesCheck(
  Supervisor supervisor, std::function<bool(SuperNodeMediator::SuperNodeInfo&)> check)
{
  map<string,string> failedNodes;

  bool success = true;
  std::string errorMessage;
  for (pair<string,SuperNodeInfo> nodePair: supervisor.nodes)
  {
    SuperNodeInfo node = get<1>(nodePair);
    //only check manifested nodes, ignore others
    if(node.manifested)
    {
      if(!node.online)
      {
        errorMessage = "check failed: node not online: " + node.name;
      }
      else if(!check(node))
      {
          string node_state =to_string((int)node.state);//string(AMLifeCycle::stateToString(node.state));
          errorMessage= "check failed: node in wrong state (" + node_state + "): " + node.name;
      }
      else if(node.status == LifeCycleStatus::ERROR)
      {
        errorMessage = "check failed: node status is ERROR: " + node.name;
      }
      if(!errorMessage.empty())
      {
        success = false;
        failedNodes.insert(pair<string,string>(node.name,errorMessage));
      }
    }//else not manifested so ignore
  }//for each node
  return pair(success,failedNodes);
}

}