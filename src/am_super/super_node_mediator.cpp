#include <am_super/super_node_mediator.h>
#include <boost/algorithm/string.hpp>


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

pair<bool, map<string, string>> SuperNodeMediator::allManifestedNodesCheck(
    Supervisor supervisor, std::function<bool(SuperNodeMediator::SuperNodeInfo&)> check)
{
  map<string, string> failed_nodes;

  bool success = true;
  std::string error_message;
  for (pair<string, SuperNodeInfo> nodePair : supervisor.nodes)
  {
    SuperNodeInfo node = get<1>(nodePair);
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