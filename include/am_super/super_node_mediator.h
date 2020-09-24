

#ifndef AM_SUPER_INCLUDE_AM_SUPER_NODE_MEDIATOR_H_
#define AM_SUPER_INCLUDE_AM_SUPER_NODE_MEDIATOR_H_
#include <ros/ros.h>
#include <brain_box_msgs/LifeCycleState.h>

#include <super_lib/am_life_cycle_types.h>
#include <super_lib/am_life_cycle.h>
#include <super_lib/am_life_cycle_mediator.h>

#include <am_super/super_state.h>
#include <am_super/super_state_mediator.h>
#include <am_super/operator_command.h>



using namespace std;
namespace am
{
class SuperNodeMediator
{
public:
  SuperNodeMediator(std::string node_name = "am_super");
  const std::string SUPER_NODE_NAME; 

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
    OperatorCommand last_op_command_received;
    
    /** True indicates the session controller has signaled the end of the session (flight, etc). */
    bool session_completed;
  };

  /**Encapsulates properties and methods that relate to the transition of states
   * from various sources (SuperState, NodeLifecycle, Flight Controller) to ensure
   * the system state is correct.
   */
  struct StateTransition
  {
    StateTransition(SuperState _to_state, std::function<bool(SuperNodeMediator::Supervisor&,SuperNodeMediator::SuperNodeInfo&)> _check,
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
    std::function<bool(SuperNodeMediator::Supervisor&,SuperNodeMediator::SuperNodeInfo&)> check;

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

    bool hasLifecycleCommand();
  };

  /**Standardizes the node name which sometimes starts with `/`.
   * @param node_name orginal name with characgters
   * @return node name stripped of characters.
   */
  std::string nodeNameStripped(std::string node_name);

  /** The only place authorized to validate a node is super.  It will call nodeNameStripped just in case.*/
  bool nodeNameIsSuper(std::string node_name);

  /** Appends super node to the manifest to participate as a lifecycle node */
  void addSuperToManifest(SuperNodeMediator::Supervisor& supervisor);

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

    /** List of node names that should receive the life_cycle_command */
    std::vector<string> failed_nodes;
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
   * @return true if Lifecyle state is inactive (already configured)*/
  bool checkReadyToArm(Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr);

  /** 
   * FIXME: this should be a private method
   * @return true if the operator sent the signal to arm the system  */
  bool checkOperatorSignaledToArm(Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr);

  /**
   * FIXME: this should be a private method
   * @return true if Lifecyle state equals ACTIVE */
  bool checkArmed(Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr);

  /**
   * FIXME: this should be a private method
   * @return true if the operator is ready to launch */
  bool checkOperatorSignaledToLaunch(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr);

  /**
   * FIXME: this should be a private method
   * @return true if the session controller signaled the end of the session (flight, etc) */
  bool checkSessionCompleted(SuperNodeMediator::Supervisor& supervisor,SuperNodeMediator::SuperNodeInfo& nr);


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
  // SuperStateMediator super_state_mediator;
  // AMLifeCycleMediator life_cycle_mediator;

  bool lifeCycleNotYetImplemented(string node_name);
  /** keyed by the current system state, if the check method passes then the new state will be the given.*/
  const std::map<SuperState, StateTransition> state_transitions_;

};
}

#endif /* AM_SUPER_INCLUDE_AM_SUPER_NODE_MEDIATOR_H_ */
