

#ifndef AM_SUPER_INCLUDE_AM_SUPER_NODE_MEDIATOR_H_
#define AM_SUPER_INCLUDE_AM_SUPER_NODE_MEDIATOR_H_
#include <ros/ros.h>
#include <brain_box_msgs/LifeCycleState.h>
#include <super_lib/am_life_cycle_types.h>
#include <super_lib/am_life_cycle.h>
#include <am_super/super_state.h>

using namespace std;
namespace am
{
class SuperNodeMediator
{
public:
  SuperNodeMediator();

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

  struct Supervisor
  {
    /** map of all nodes in the system*/
    map<string, SuperNodeInfo> nodes;

    /** manifest node (generated from manifest param) */
    std::vector<string> manifest;

    /**
     * system state
     */
    SuperState system_state_;

  };

  /**Standardizes the node name which sometimes starts with `/`.
   * @param node_name orginal name with characgters
   * @return node name stripped of characters.
   */
  std::string nodeNameStripped(std::string node_name);

  /**Nodes declared in manifest are created with default
   * state so the system can seek them out.
   *
   * @return node info with given name and default information
   */
  SuperNodeMediator::SuperNodeInfo initializeManifestedNode(std::string node_name);

  /**@return true if Lifecyle state is ready to be configured */
  static bool checkReadyForConfigureState(SuperNodeMediator::SuperNodeInfo& nr);

  /**@return true if Lifecyle state is ready to be activated */
  static bool checkReadyForActivateState(SuperNodeMediator::SuperNodeInfo& nr);

  /**@return true if Lifecyle state equals Activate */
  static bool checkActivateState(SuperNodeMediator::SuperNodeInfo& nr);

  /** Reads the given manifest string, typically provided by a ROS param, 
   * converts it to a vector or node names which will be assigned to the given 
   * Supervisor.
   * 
   * @param supervisor maintaing state receiving node names in the manifest
   * @param manifest comma separate list of node names
   */
  void parseManifest(Supervisor &supervisor, string manifest);

  /**
     * check if all manifested nodes are ready for configuration.
     * The manifest indicates the node is necessary for operation.
     * 
     * This means:
     * - all are online
     * - all states are UNCONFIGURED or INACTIVE or ACTIVE
     * - all statuses are not error
     * 
     * @param supervisor with the state of the system to be checked
     * @param check function that will be called with each node registered with Supervisor
     * @return a pair with overall success and a map containing any erroneous node names with message explaining why
     *
     */
  pair<bool, map<string, string>> allManifestedNodesCheck(Supervisor supervisor,
                                                          function<bool(SuperNodeMediator::SuperNodeInfo&)> check);

  /**@return the number of nodes where online=true*/
  int nodesOnlineCount(Supervisor supervisor);
  
  /**@return the number of nodes where online=true and manifested=true*/
  int manifestedNodesOnlineCount(Supervisor supervisor);

private:
};
}

#endif /* AM_SUPER_INCLUDE_AM_SUPER_NODE_MEDIATOR_H_ */
