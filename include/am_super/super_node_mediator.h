

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

  /**
   * Instructions Super receives from flight controller.
   */
  enum SuperFltCtrlState
  {
    INIT,
    AUTO,
    HOLD
  };

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
     * Overall state of the system, cumulative of the nodes
     * and super together.
     */
    SuperState system_state;

    /**
     * Allows super to manage flight control.
     * DEPRECATED - Remove this when operator_is_ready is complete AM-461
     */
    SuperFltCtrlState flt_ctrl_state;

    /** Indication that the operator is supervising the robot, has sent the signal to arm the system */
    bool operator_is_ready_to_arm;
    bool operator_is_ready_to_launch;
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

  /**Provided by transitionReady method used by the node to trnasition states and send signals according
   * the properties within.
   */
  struct TransitionInstructions
  {
    /**System state should become new_state*/
    bool ready_for_transition;
    /**The new system state if ready_for_transition*/
    SuperState new_state;
    /** if True, then command should be sent.  if ready_for_transition=true, then this is false*/
    bool resend_life_cycle_command;

    /** The command that notifies nodes to continue processing so the state can transition*/
    LifeCycleCommand life_cycle_command;
  };

  /**Provides the next state when the system is in a specific state provided by the Supervisor.
   * https://automodality.atlassian.net/wiki/spaces/AMROS/pages/929234949/AMROS+System+States
   *
   * @param supervisor is in charge of knowing the state of the system
   * @return pair with the boolean indicating transition is ready and the optional state if ready.
   */
  SuperNodeMediator::TransitionInstructions transitionReady(Supervisor supervisor);

  /**
   * Called when transitionReady state fails to provide a LifeCycleCommand for those states
   * that notify other nodes to keep trying (early in the lifecycle).
   * @return true with the lifecycle if needed or false with no lifecycle if not needed.
   */
  pair<bool, LifeCycleCommand> lifeCycleCommand(SuperState system_state);

  /**
   * FIXME: this should be a private method
   * @return true if Lifecyle state is ready to be configured */
  static bool checkReadyForConfigureState(Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr);

  /**Delegate method used by allManifestedNodesCheck to see if manifested nodes have the state required for arming.
   * FIXME: this should be a private method
   * @return true if nodes are inactive, active and the operator is ready to arm  */
  static bool checkReadyToArm(Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr);

  /**
   * FIXME: this should be a private method
   * @return true if Lifecyle state equals Activate */
  static bool checkActivateState(Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr);

  /**
   * FIXME: this should be a private method
   * @return true if nodes are active and the operator is ready to launch */
  static bool checkArmedToAuto(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr);


  /** Reads the given manifest string, typically provided by a ROS param,
   * converts it to a vector or node names which will be assigned to the given
   * Supervisor.
   *
   * @param supervisor maintaing state receiving node names in the manifest
   * @param manifest comma separate list of node names
   */
  void parseManifest(Supervisor& supervisor, string manifest);

  /**
     * check if all manifested nodes are ready for configuration.
     * The manifest indicates the node is necessary for operation.
     *
     * This means:
     * - all are online
     * - all states are UNCONFIGURED or INACTIVE or ACTIVE
     * - all statuses are not error
     * FIXME: this method should be private
     * @param supervisor with the state of the system to be checked
     * @param check function that will be called with each node registered with Supervisor
     * @return a pair with overall success and a map containing any erroneous node names with message explaining why
     *
     */
  pair<bool, map<string, string>> allManifestedNodesCheck(Supervisor& supervisor,
                                                          function<bool(Supervisor&,SuperNodeMediator::SuperNodeInfo&)> check);

  /**@return the number of nodes where online=true*/
  int nodesOnlineCount(Supervisor supervisor);

  /**@return the number of nodes where online=true and manifested=true*/
  int manifestedNodesOnlineCount(Supervisor supervisor);

  /** @return the nodes declared in the manifest, but online = false*/
  map<string,SuperNodeMediator::SuperNodeInfo> manifestedNodesNotOnline(Supervisor supervisor);

  /** @return a csv string of names of the manifested nodes not online */
  string manifestedNodesNotOnlineNamesList(Supervisor supervisor);

private:
};
}

#endif /* AM_SUPER_INCLUDE_AM_SUPER_NODE_MEDIATOR_H_ */
