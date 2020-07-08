#include <am_super/super_node_mediator.h>



namespace am
{





SuperNodeMediator::SuperNodeMediator(){

}

std::string SuperNodeMediator::nodeNameStripped(std::string node_name){
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

}