#include <am_super/super_node_mediator.h>

namespace am
{
struct SuperNodeInfo
{
  std::string name;        // node name in ROS
  int pid;                 // process id of node
  float cpu_usage;         // amount of cpu node is consuming
  float gpu_usage;         // amount of gpu node is consuming
  float mem_usage;         // amount of memory node is consuming
  LifeCycleState state;    // https://index.ros.org/p/lifecycle/
  LifeCycleStatus status;  // node lifecycle status
  bool manifested;         // nodes was in manfiest
  bool online;             // node is online
  ros::Time last_contact;  // last time a message was received from the node
};

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
}