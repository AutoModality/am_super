

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
#include <am_super/controller_state.h>


using namespace std;
namespace am
{
class SuperNodeMediator
{
public:
  SuperNodeMediator(const std::string& node_name);
  
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

 /**
  * Describes the platform configuration for the hardware running AMROS.
  *  maker-model-app
  * Some variants are just the maker, some maker-model.
  * Sometimes a variant may provide just the app, indiciating the app could be flown on different platforms.
  */ 
  struct PlatformVariant
  {
    /** the manufacture of the drone (DJI, ACSL)*/
    std::string maker;
    /** The product name of the drone specific to the maker (m210,m300,mini,pf2)*/
    std::string model;
    /** The application configuration for the mission (bridge, subt, etc). */
    std::string app;

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
    
    /** Last state of the controller received */
    ControllerState last_controller_state_received;

    /** Signals if any of the manifested nodes status errored */
    bool status_error = false;
  };

  /**Encapsulates properties and methods that relate to the transition of states
   * from various sources (SuperState, NodeLifecycle, Flight Controller) to ensure
   * the system state is correct.
   */
  struct StateTransition
  {
    static const SuperState NO_SUPER_STATE = (SuperState)-1;
    static const LifeCycleCommand NO_LIFECYCLE_COMMAND = (LifeCycleCommand)-1;
    static const OperatorCommand NO_OPERATOR_COMMAND = (OperatorCommand)-1;
    static const ControllerState NO_CONTROLLER_STATE = (ControllerState)-1;

    StateTransition(SuperState _to_state = NO_SUPER_STATE, std::function<bool(SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> _check = NULL,
                    LifeCycleCommand _life_cycle_command = NO_LIFECYCLE_COMMAND, OperatorCommand _operator_command = NO_OPERATOR_COMMAND, 
                    ControllerState _controller_state = NO_CONTROLLER_STATE)
    {
      to_state = _to_state;
      check = _check;
      life_cycle_command = _life_cycle_command;
      operator_command = _operator_command;
      on_check_result = true; //TODO: remove this; we are assuming the check method should always return true now
      controller_state = _controller_state;
    }
    /**The future Supervisor.systemState if checks pass.*/
    SuperState to_state;
    /**Function that indicates if the transition is allowed (based on node lifecycle)*/
    std::function<bool(SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check;

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

    /** The command super needs to receive in order for us to attempt this transition*/
    OperatorCommand operator_command;

    /** The controllerState super needs to receive in order for us to attempt this transition */
    ControllerState controller_state;
  };

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
    /** List of reasons nodes aren't transitioning */
    std::vector<string> failed_nodes_reasons;
  };

  /** Returns the name of the node that is using the mediator */
  std::string_view getNodeName();

  /**Standardizes the node name which sometimes starts with `/`.
   * @param node_name orginal name with characgters
   * @return node name stripped of characters.
   */
  //FIXME: move static method to am_utils
  static std::string nodeNameStripped(std::string node_name);

  /** The only place authorized to validate a node is super.  It will call nodeNameStripped just in case.*/
  bool nodeNameIsSuper(std::string node_name);

  /** Appends super node to the manifest to participate as a lifecycle node */
  void addSuperToManifest(SuperNodeMediator::Supervisor& supervisor);

  /** Sets the supervisors last operator command received */
  void setOperatorCommand(SuperNodeMediator::Supervisor& supervisor, const OperatorCommand& command);

  /** Stores the last state reported by the controller node into supervisor */
  void setControllerState(SuperNodeMediator::Supervisor& supervisor, const ControllerState& controller_state);

  /**Nodes declared in manifest are created with default
   * state so the system can seek them out.
   *
   * @return node info with given name and default information
   */
  SuperNodeMediator::SuperNodeInfo initializeManifestedNode(std::string node_name);

  /**Provides the next state when the system is in a specific state provided by the Supervisor.
   * https://automodality.atlassian.net/wiki/spaces/AMROS/pages/929234949/AMROS+System+States
   *
   * @param supervisor is in charge of knowing the state of the system
   * @return pair with the boolean indicating transition is ready and the optional state if ready.
   */
  SuperNodeMediator::TransitionInstructions transitionReady(Supervisor supervisor);

  /** 
   * FIXME: currently public for unit testing
   * @return the transition that we will attempt. If no transition was found, invalidTransition() is returned
   */
  StateTransition getStateTransition(const Supervisor &supervisor);

  /** returns a state transition that should be attempted when a LifeCycleError has occured. */
  StateTransition getErrorTransition();

  /**
   * @return Default StateTransition with all of its fields at default. A transition is valid if it's values aren't default
   */
  StateTransition invalidTransition();

  /**
   * Called when transitionReady state fails to provide a LifeCycleCommand for those states
   * that notify other nodes to keep trying (early in the lifecycle).
   * @return true with the lifecycle if needed or false with no lifecycle if not needed.
   */
  pair<bool, LifeCycleCommand> lifeCycleCommand(SuperState system_state);


  /**
  * FIXME: this should be a private method
   * @return true if Lifecyle state is inactive (already configured)*/
  static bool checkReadyToArm(SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator);

  /**
   * FIXME: this should be a private method
   * @return true if Lifecyle state equals ACTIVE */
  static bool checkArmed(SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator);

  /**
   * FIXME: this should be a private method
   * @return true if Lifecyle state equals ACTIVE or INACTIVE */
  static bool checkNodesActiveOrInactive(SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator);

  /**
   * FIXME: this should be a private method
   * @return true if Lifecyle state equals ShuttingDown or Finalized */
  static bool checkNodesShuttingDownOrFinalized(SuperNodeMediator::SuperNodeInfo& nr, SuperNodeMediator& node_mediator);


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
                                                          function<bool(SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check);

  /**@return the number of nodes where online=true*/
  int nodesOnlineCount(Supervisor supervisor);

  /**@return the number of nodes where online=true and manifested=true*/
  int manifestedNodesOnlineCount(Supervisor supervisor);

  /** @return the nodes declared in the manifest, but online = false*/
  map<string,SuperNodeMediator::SuperNodeInfo> manifestedNodesNotOnline(Supervisor supervisor);

  /** @return a csv string of names of the manifested nodes not online */
  string manifestedNodesNotOnlineNamesList(Supervisor supervisor);

  /**
   * @returns true - operator_command has been assigned 
   */
  bool transitionHasOperatorCommand(const StateTransition&);

  /**
   * @returns true - life_cycle_command has been assigned
   */
  bool transitionHasLifecycleCommand(const StateTransition&);

  /**
   * @returns true - life_cycle_command has been assigned
   */
  bool transitionHasControllerState(const StateTransition&);

  /**
   * @returns true - to_state has been assigned
   */
  bool transitionIsValid(const StateTransition&);

  /**
   * @returns true - if we are allowed to transition to this state regardless of the state we are currently in
   */
  bool forceTransition(const SuperState& to_state);

  /**
   * @returns true if the running platform has matching components for that required
   */
  bool isCorrectPlatform(const PlatformVariant &required, const PlatformVariant &actual);

  /** Given the string in the configurations, the variant is returned with the components filled in */
  PlatformVariant platformConfigToVariant(const std::string config);

private:
  /** name of supervisor node */
  const std::string SUPER_NODE_NAME;

  /** provides LifeCycleMediator methods */
  AMLifeCycleMediator life_cycle_mediator;

  /** keyed by the current system state, if the check method passes then the new state will be the given.*/
  const std::map<SuperState, std::map<SuperState, StateTransition>> state_transitions_ ;


  /** @brief temporary hack to allow manifested nodes to not halt transitions.*/
  bool lifeCycleNotYetImplemented(string node_name);

  
};
}

#endif /* AM_SUPER_INCLUDE_AM_SUPER_NODE_MEDIATOR_H_ */
