#include <am_super/super_node_mediator.h>
#include <ros/ros.h>

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
}