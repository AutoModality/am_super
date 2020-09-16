#include <functional>
#include <memory>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud2.h>


#include <am_super/baby_sitter.h>
#include <am_super/super_state.h>
#include <am_super/super_state_mediator.h>
#include <am_super/super_node_mediator.h>

#include <brain_box_msgs/BlinkMCommand.h>
#include <brain_box_msgs/LifeCycleState.h>
#include <brain_box_msgs/LogControl.h>
#include <brain_box_msgs/OperatorCommand.h>
#include <brain_box_msgs/StampedAltimeter.h>
#include <brain_box_msgs/Super2Status.h>
#include <brain_box_msgs/VxState.h>
#include <brain_box_msgs/ControllerState.h>

#include <am_super/controller_state.h>

#include <super_lib/am_life_cycle_types.h>
#include <super_lib/am_life_cycle.h>
#include <super_lib/am_life_cycle_mediator.h>
#include <super_lib/am_super_topics.h>

#include <vb_util_lib/bag_logger.h>
#include <vb_util_lib/topics.h>
#include <vb_util_lib/trace.h>
#include <vb_util_lib/vb_main.h>
#if CUDA_FLAG
#include <cuda/cuda_utility_class.h>
#endif

using namespace std;

namespace am
{
/**
 *  AM supervisor class. aggregates system state and system health and manages node lifecycle.
 *
 *  uses BabySitter instances to generate state and health for nodes that don't publish brain_box_msgs::LifeCycleState
 */
class AMSuper : AMLifeCycle
{
private:
  /** 
   * heartbeat log output period
   */
  const int LOG_THROTTLE_S = 10;

  /**
   * the ros node handle
   */
  ros::NodeHandle nh_;

  /*
   * see constructor for details
   */
  ros::Publisher lifecycle_pub_;
  ros::Publisher vstate_summary_pub_;
  ros::Publisher super_status_pub_;
  ros::Publisher led_pub_;
  ros::Subscriber node_state_sub_;
  ros::Subscriber node_status_sub_;
  ros::Subscriber operator_command_sub_;
  ros::Subscriber controller_state_sub;
  ros::Timer heartbeat_timer_;

  /** manage logic for SuperState transitions */
  SuperStateMediator state_mediator_;

  /* manage logic for LifeCycle */
  AMLifeCycleMediator life_cycle_mediator_;

  /** Node behavior management.*/
  SuperNodeMediator node_mediator_;

  /** The current state of the system. */
  SuperNodeMediator::Supervisor supervisor_;

  /**
   * amount of time in seconds without hearing from a node that will cause it to go offline
   */
  double node_timeout_s_;

  /**
   * baglogger level
   */
  const int SU_LOG_LEVEL = 1;

  //
  // babysitters
  //
  const std::string NODE_BS_ALTIMETER = "can_node";  // TODO: replace with system global const
 
  typedef brain_box_msgs::StampedAltimeter altimeter_bs_msg_type;
  am::BabySitter<altimeter_bs_msg_type>* altimeter_bs_;
  const std::string ALTIMETER_BS_TOPIC = "/sensor/distance/agl_lw";  // TODO: replace with system global const
  const int ALTIMETER_HZ = 20;

  const std::string NODE_BS_DJI = "dji_sdk";  // TODO: replace with system global const
  typedef sensor_msgs::Joy dji_bs_msg_type;
  am::BabySitter<dji_bs_msg_type>* dji_bs_;
  const std::string DJI_BS_TOPIC = "/dji_sdk/rc";  // TODO: replace with system global const
  const int DJI_HZ = 50;

#if CUDA_FLAG
  std::shared_ptr<am::CudaUtility> gpu_info_;
#endif

public:
  AMSuper() : nh_("~")
  {
    ROS_INFO_STREAM(NODE_FUNC);

    ros::param::param<double>("~node_timeout_s", node_timeout_s_, 2.0);
    ROS_INFO_STREAM("node_timeout_s = " << node_timeout_s_);

    /*
     * create initial node list from manifest and create babysitters as needed
     */
    supervisor_.system_state = SuperState::OFF;
    // strip spaces from manifest param
    string manifest_param;
    ros::param::param<string>("~manifest", manifest_param, "");

    node_mediator_.parseManifest(supervisor_, manifest_param);

    //Add super to manifest
    node_mediator_.addSuperToManifest(supervisor_);

    // if a manifest has been specified
    if (!supervisor_.manifest.empty())
    {
      ROS_INFO_STREAM("configuring nodes from manifest: " << manifest_param);
      for (string& name : supervisor_.manifest)
      {
        // create a new node in the list for each name in manifest
        SuperNodeMediator::SuperNodeInfo nr = node_mediator_.initializeManifestedNode(name);
        supervisor_.nodes.insert(pair<string, SuperNodeMediator::SuperNodeInfo>(name, nr));
        ROS_INFO_STREAM("  " << name);

        // create babysitters based on hard coded node names
        if (!name.compare(NODE_BS_ALTIMETER))
        {
          int altimeter_warn_ms, altimeter_error_ms;
          calcBSTiming(ALTIMETER_HZ, altimeter_warn_ms, altimeter_error_ms);
          altimeter_bs_ = new am::BabySitter<altimeter_bs_msg_type>(
              nh_, BagLogger::instance(), name, ALTIMETER_BS_TOPIC, altimeter_warn_ms, altimeter_error_ms);
        }
        else if (!name.compare(NODE_BS_DJI))
        {
          int dji_warn_ms, dji_error_ms;
          calcBSTiming(DJI_HZ, dji_warn_ms, dji_error_ms);
          dji_bs_ = new am::BabySitter<dji_bs_msg_type>(nh_, BagLogger::instance(), name, DJI_BS_TOPIC, dji_warn_ms,
                                                        dji_error_ms);
        }
      }
    }
    else
    {
      ROS_WARN_STREAM("Manifest is empty.  No nodes will be monitored.");
    }
    
    reportSystemState();

#if CUDA_FLAG
    ROS_INFO("##########GPU Monitoring is ON##########");
    gpu_info_ = std::make_shared<am::CudaUtility>(nh_);
#endif

    /**
     * system status pub
     */
    vstate_summary_pub_ = nh_.advertise<brain_box_msgs::VxState>("/vstate/summary", 1000);
    /**
     * node lifecycle state pub. used to tell nodes to change their lifecycle state.
     */
    lifecycle_pub_ = nh_.advertise<brain_box_msgs::LifeCycleCommand>("/node_lifecycle", 100);
    /**
     * led control pub
     */
    led_pub_ = nh_.advertise<brain_box_msgs::BlinkMCommand>(am::am_topics::LED_BLINK, 1000);
    /**
     * super status contains online naode list for gcs_comms
     */
    super_status_pub_ = nh_.advertise<brain_box_msgs::Super2Status>("/super/status", 1000);

    supervisor_.system_state = SuperState::BOOTING;
    supervisor_.flt_ctrl_state = SuperNodeMediator::SuperFltCtrlState::INIT;

    BagLogger::instance()->startLogging("SU", SU_LOG_LEVEL);

    // subs should always come at the end
    
    /**
     * node status via LifeCycle
     */
    node_state_sub_ = nh_.subscribe("/node_state", 100, &AMSuper::nodeStateCB, this);
    /**
     * legacy node status
     */
    node_status_sub_ = nh_.subscribe("/process/status", 100, &AMSuper::statusCB, this);

    /**
     * commands from operator
     */
    operator_command_sub_ = nh_.subscribe(am_super_topics::OPERATOR_COMMAND, 100, &AMSuper::operatorCommandCB, this);

    controller_state_sub = nh_.subscribe(am_super_topics::CONTROLLER_STATE, 100, &AMSuper::controllerStateCB, this);

    heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0), &AMSuper::heartbeatCB, this);
  }

  ~AMSuper()
  {
    // TODO: need a clean way of shutting down. i don't think ctrl-c calls this and
    // certainly power off doesn't. the best design is to have a power-off sensor
    // that sends a signal of some sort (i.e. publishes a message).
    BagLogger::instance()->stopLogging();
  }

private:
  /**
   * process LifeCycleState messages from nodes
   *
   * LifeCycle messages are sent once a second by the LifeCycle heartbeat, but may
   * come more frequently if a node chooses.
   */
  void nodeStateCB(const ros::MessageEvent<brain_box_msgs::LifeCycleState const>& event)
  {
    const brain_box_msgs::LifeCycleState::ConstPtr& rmsg = event.getMessage();

    /*
     * process the message
     *
     * TODO: this should probably be more reactive. i.e. if a node sends a non-ACTIVE state
     * or a non-OK status, we should probably react to it immediately rather than wait
     * for the timeout.
     */
    processState(rmsg->node_name, (LifeCycleState)(rmsg->state), (LifeCycleStatus)(rmsg->status), rmsg->subsystem,
                 rmsg->value, rmsg->process_id, event.getReceiptTime());

    // TODO: topic name should come from vb_util_lib::topics.h
    LOG_MSG("/node_state", rmsg, SU_LOG_LEVEL);
  }

  /**
   * process legacy messages from nodes
   * TODO: mark deprecated due to legacy. use nodeStateCB.
   */
  void statusCB(const ros::MessageEvent<brain_box_msgs::NodeStatus const>& event)
  {
    const brain_box_msgs::NodeStatus::ConstPtr& rmsg = event.getMessage();

    /*
     * legacy messages don't carry any state or status info so just process as ACTIVE/OK
     */
    processState(rmsg->node_name, LifeCycleState::INACTIVE, LifeCycleStatus::OK, rmsg->status, rmsg->value,
                 rmsg->process_id, event.getReceiptTime());

    // TODO: topic name should come from vb_util_lib::topics.
    LOG_MSG("/process/status", rmsg, SU_LOG_LEVEL);
  }

  void controllerStateCB(const ros::MessageEvent<brain_box_msgs::ControllerState const>& event)
  {
    const brain_box_msgs::ControllerState::ConstPtr& rmsg = event.getMessage();

    switch(rmsg->state)
    {
      case brain_box_msgs::ControllerState::COMPLETED:
        ROS_INFO_STREAM("Controler node: " << rmsg->node_name << " state is COMPLETED");
        supervisor_.session_completed = true;
        break;
    }
  }

  void operatorCommandCB(const ros::MessageEvent<brain_box_msgs::OperatorCommand const>& event)
  {
    const brain_box_msgs::OperatorCommand::ConstPtr& rmsg = event.getMessage();
    
    ROS_INFO_STREAM(rmsg->node_name << " sent command " << rmsg->command );
    switch(rmsg->command)
    {
      case brain_box_msgs::OperatorCommand::ARM:
        supervisor_.last_op_command_received = OperatorCommand::ARM;
        break;
      case brain_box_msgs::OperatorCommand::LAUNCH:
        supervisor_.last_op_command_received = OperatorCommand::LAUNCH;
        break;
    }
    LOG_MSG(am_super_topics::OPERATOR_COMMAND, rmsg, SU_LOG_LEVEL);
  }
  /**
   * process state
   * @param node_name_in
   * @param state
   * @param status
   * @param subsystem legacy mechanism for flight controller to report it's state
   * @param value legacy mechanism for flight controller to report it's state
   * @param pid
   * @param last_contact
   */
  void processState(const std::string& node_name_in, const am::LifeCycleState state, const am::LifeCycleStatus status,
                    const std::string& subsystem, const std::string& value, const int pid,
                    const ros::Time& last_contact)
  {
    // strip leading '/' from the node name if needed
    string node_name = node_mediator_.nodeNameStripped(node_name_in);

    // search for the node in the list
    bool nodes_changed = false;
    map<string, SuperNodeMediator::SuperNodeInfo>::iterator it;
    it = supervisor_.nodes.find(node_name);
    if (it != supervisor_.nodes.end())
    {
      // if we get here, the node is already in our list
      SuperNodeMediator::SuperNodeInfo& nr = it->second;
      if (!nr.online)
      {
        ROS_INFO_STREAM("manifested node '" << node_name << "' came online");
        nr.online = true;
        nodes_changed = true;
      }
      if (nr.state != state)
      {
        ROS_INFO_STREAM(node_name << " changed state to = " << life_cycle_mediator_.stateToString(state));
        nr.state = state;
        nodes_changed = true;
      }
      if (nr.status != status)
      {
        ROS_INFO_STREAM(node_name << " changed status to = " << life_cycle_mediator_.statusToString(status));
        nr.status = status;
        nodes_changed = true;
      }
      if (nr.pid != pid)
      {
        ROS_WARN_STREAM(node_name << " changed process id to = " << pid);
        nr.pid = pid;
        nodes_changed = true;
      }
      nr.last_contact = last_contact;
    }
    else
    {
      // if we get here, the node is not in the manifest and we've never heard from it before
      ROS_WARN_STREAM("unknown node " << node_name << " came online. state: " << life_cycle_mediator_.stateToString(state)
                                      << ", status: " << life_cycle_mediator_.statusToString(status));
      SuperNodeMediator::SuperNodeInfo nr;
      nr.name = node_name;
      nr.pid = pid;
      nr.online = true;
      nr.last_contact = last_contact;
      nr.manifested = false;
      nr.state = state;
      nr.status = status;
      supervisor_.nodes.insert(pair<string, SuperNodeMediator::SuperNodeInfo>(node_name, nr));
      nodes_changed = true;
    }

    if (nodes_changed)
    {
      reportSystemState();
      checkForSystemStateTransition();
    }

    // cache flight controller state and check for state transition
    if (!node_name.compare("flight_controller") && !subsystem.compare("FLIGHT_CONTROL"))
    {
      bool flt_ctrl_state_changed = false;
      if (!value.compare("AUTO") && supervisor_.flt_ctrl_state != SuperNodeMediator::SuperFltCtrlState::AUTO)
      {
        supervisor_.flt_ctrl_state = SuperNodeMediator::SuperFltCtrlState::AUTO;
        flt_ctrl_state_changed = true;
      }
      else if (!value.compare("HOLD") && supervisor_.flt_ctrl_state != SuperNodeMediator::SuperFltCtrlState::HOLD)
      {
        supervisor_.flt_ctrl_state = SuperNodeMediator::SuperFltCtrlState::HOLD;
        flt_ctrl_state_changed = true;
      }
      if (flt_ctrl_state_changed)
      {
        ROS_INFO_STREAM_THROTTLE(1.0, "flight status: " << value);
        checkForSystemStateTransition();
      }
    }
  }

  /**
   * called once per second.
   *
   * times out nodes that haven't been heard from recently. reports on status to bag and trace logs.
   */
  void heartbeatCB(const ros::TimerEvent& event) override
  {
#if CUDA_FLAG
    gpu_info_->display();
#endif
    brain_box_msgs::VxState state_msg;
    state_msg.state = (uint8_t)supervisor_.system_state;
    vstate_summary_pub_.publish(state_msg);

    // cycle thru all the nodes in the list to look for a timeout
    ros::Time now = ros::Time().now();
    map<string, SuperNodeMediator::SuperNodeInfo>::iterator it;
    for (it = supervisor_.nodes.begin(); it != supervisor_.nodes.end(); it++)
    {
      SuperNodeMediator::SuperNodeInfo& nr = (*it).second;
      if (nr.online)
      {
        ros::Duration time_since_contact = now - nr.last_contact;
        ros::Duration timeout_dur(node_timeout_s_);
        if (time_since_contact > timeout_dur)
        {
          nr.online = false;
          ROS_ERROR_STREAM("node timed out:" << nr.name);
          reportSystemState();
        }
      }
    }

    // check for state transition due to timeouts or anything else that changed since last heartbeat
    checkForSystemStateTransition();

    int num_manifest_nodes_online = node_mediator_.manifestedNodesOnlineCount(supervisor_);
    // publish and bag log super status message
    brain_box_msgs::Super2Status status_msg;
    status_msg.man = supervisor_.manifest.size();
    status_msg.man_run = num_manifest_nodes_online;
    status_msg.run = node_mediator_.nodesOnlineCount(supervisor_);

    for (it = supervisor_.nodes.begin(); it != supervisor_.nodes.end(); it++)
    {
      SuperNodeMediator::SuperNodeInfo& nr = (*it).second;
      status_msg.nodes.push_back(nr.name);
    }
    LOG_MSG("/status/super", status_msg, 1);
    if (super_status_pub_.getNumSubscribers() > 0)
    {
      super_status_pub_.publish(status_msg);
    }

    // report current status to trace log
    std::stringstream ss;
    genSystemState(ss);

    if (supervisor_.manifest.size() != num_manifest_nodes_online || supervisor_.system_state == SuperState::ABORT ||
        supervisor_.system_state == SuperState::HOLD || supervisor_.system_state == SuperState::MANUAL)
    {
      // if all manifested nodes aren't running, report as error
      ROS_ERROR_STREAM(ss.str());
      ROS_ERROR_STREAM("not online: " << node_mediator_.manifestedNodesNotOnlineNamesList(supervisor_));
    }
    else
    {
      // if all manifested nodes are running, report as info
      ROS_INFO_STREAM_THROTTLE(LOG_THROTTLE_S, ss.str());
    }
    AMLifeCycle::heartbeatCB(event);
  }

  /**
   * update stream with system state and status
   */
  void genSystemState(std::stringstream& ss)
  {
    int num_manifest_nodes_online = node_mediator_.manifestedNodesOnlineCount(supervisor_);
    int num_nodes_online = node_mediator_.nodesOnlineCount(supervisor_);
    ss << "state: " << state_mediator_.stateToString(supervisor_.system_state)
       << ", manifest: " << supervisor_.manifest.size() << ", manifest online:" << num_manifest_nodes_online
       << ", total online:" << num_nodes_online;
  }

  /**
   * report system state and status to trace log
   */
  void reportSystemState()
  {
    std::stringstream ss;
    genSystemState(ss);
    ROS_INFO_STREAM(ss.str());
  }

  /**
   * send lifecycle message to nodes
   * @param node_name "" = broadcast
   * @param state
   *
   * TODO: should throttle any given node_name/state combo to 1/sec.
   */
  void sendLifeCycleCommand(const std::string_view& node_name, const LifeCycleCommand command)
  {
    ROS_DEBUG_STREAM("sending command: " << life_cycle_mediator_.commandToString(command) << " to " << node_name << " lifecycle");
    brain_box_msgs::LifeCycleCommand msg;
    msg.node_name = node_name;
    msg.command = (brain_box_msgs::LifeCycleCommand::_command_type)command;
    lifecycle_pub_.publish(msg);
  }

  /**
   * check if all manifested nodes are ready for configuration
   * @param state
   * @param status
   * @return true if all manifested nodes are ready to become active
   *
   * This means:
   * - all are online
   * - all states are UNCONFIGURED or INACTIVE or ACTIVE
   * - all statuses are not error
   */
  bool allManifestedNodesCheck(std::function<bool(SuperNodeMediator::Supervisor&, SuperNodeMediator::SuperNodeInfo&)> check)
  {
    pair<bool, map<string, string>> result = node_mediator_.allManifestedNodesCheck(supervisor_, check);
    bool success = result.first;
    if (!success)
    {
      for (const auto & [ node_name, error_message ] : result.second)
      {
        ROS_WARN_STREAM(error_message);
      }
    }
    return success;
  }

  /**
   * check for state transition based upon current state and values of member fields.
   * Will call to modify the system state if transition is necessary. Will also call
   * lifecycle command if indicated to do so based on the state.
   * See SuperNodeMediator::TransitionInstructions
   */
  void checkForSystemStateTransition()
  {
    if(getState() == LifeCycleState::INACTIVE) //if super lifecycle is currently inactive
    {
      sendLifeCycleCommand(SuperNodeMediator::SUPER_NODE_NAME, LifeCycleCommand::ACTIVATE);
    }
    else
    {
      // ask the mediator to check with the supervisor
      SuperNodeMediator::TransitionInstructions transition_instructions = node_mediator_.transitionReady(supervisor_);
      if (transition_instructions.ready_for_transition)
      {
        setSystemState(transition_instructions.new_state);
      }
      else if (transition_instructions.resend_life_cycle_command)
      {
        LifeCycleCommand command = transition_instructions.life_cycle_command;
        std::string failed_nodes_string = boost::algorithm::join(transition_instructions.failed_nodes, ", ");
        ROS_INFO_STREAM(state_mediator_.stateToString(supervisor_.system_state)
                        << ": sending " << life_cycle_mediator_.commandToString(command) << " to "
                        << failed_nodes_string);

        for(string failed_node_name : transition_instructions.failed_nodes)
        {
          sendLifeCycleCommand(failed_node_name, command);
        }
      }      
    }
  }

  /**
   * set the system state
   *
   * perform any actions associated with each state transition.
   *
   * publish the current led colors and system state
   */
  void setSystemState(SuperState state)
  {
    ROS_INFO_STREAM(state_mediator_.stateToString(supervisor_.system_state) << " --> "
                                                                            << state_mediator_.stateToString(state));

    bool legal = state_mediator_.allowsTransition(supervisor_.system_state, state);

    if (!legal)
    {
      ROS_ERROR_STREAM("illegal state transition from " << state_mediator_.stateToString(supervisor_.system_state)
                                                        << " to " << state_mediator_.stateToString(state));
    }
    else
    {

      // persist given state as the new current state
      supervisor_.system_state = state;

      if(supervisor_.system_state == SuperState::AUTO) //session just started when we enter AUTO mode
      {
        supervisor_.session_completed = false;
      }

      reportSystemState();

      sendLEDMessage();

      brain_box_msgs::VxState state_msg;
      state_msg.state = (uint8_t)supervisor_.system_state;
      vstate_summary_pub_.publish(state_msg);
    }
  }

  /**
   * send led color message based on raw values
   */
  void sendLEDMessage(int r, int g, int b, float period = 0.0)
  {
    brain_box_msgs::BlinkMCommand led_msg;
    led_msg.rgb.red = r;
    led_msg.rgb.green = g;
    led_msg.rgb.blue = b;
    led_msg.blink_rate = period;
    led_pub_.publish(led_msg);
  }

  static constexpr double LED_SOLID = 0.0;
  static constexpr double LED_BLINK_FAST = 1.0;
  static constexpr double LED_BLINK_SLOW = 2.0;
  /**
   * send led color message based on system state
   */
  void sendLEDMessage()
  {
    int r = 0, b = 0, g = 0;
    float rate = 0.0;

    switch (supervisor_.system_state)
    {
      case SuperState::OFF:
        r = 0;
        g = 0;
        b = 0;
        rate = LED_SOLID;
        break;
      case SuperState::BOOTING:
        r = 0;
        g = 0;
        b = 255;
        rate = LED_BLINK_SLOW;
        break;
      case SuperState::READY:
        r = 0;
        g = 0;
        b = 255;
        rate = LED_SOLID;
        break;
      case SuperState::ARMING:
        // TODO: should be green/blue slow. probably need to change can node
        r = 0;
        g = 0;
        b = 255;
        rate = LED_SOLID;
        break;
      case SuperState::ARMED:
        // TODO: should be green/blue fast. probably need to change can node
        r = 0;
        g = 0;
        b = 255;
        rate = LED_SOLID;
        break;
      case SuperState::AUTO:
        r = 0;
        g = 255;
        b = 0;
        rate = LED_SOLID;
        break;
      case SuperState::SEMI_AUTO:
        r = 0;
        g = 255;
        b = 0;
        rate = LED_BLINK_SLOW;
        break;
      case SuperState::HOLD:
        r = 255;
        g = 0;
        b = 0;
        rate = LED_BLINK_SLOW;
        break;
      case SuperState::ABORT:
        r = 255;
        g = 0;
        b = 0;
        rate = LED_BLINK_FAST;
        break;
      case SuperState::MANUAL:
        r = 255;
        g = 0;
        b = 0;
        rate = LED_SOLID;
        break;
      case SuperState::SHUTDOWN:
        r = 0;
        g = 0;
        b = 255;
        rate = LED_BLINK_FAST;
        break;
    }

    sendLEDMessage(r, g, b, rate);
  }

  /**
   * calculate babysitter timing params
   * @param hz expected frequency in hz
   * @param warn_ms calculated warning period in milliseconds
   * @param error_ms calculated error period in milliseconds
   */
  void calcBSTiming(int hz, int& warn_ms, int& error_ms)
  {
    warn_ms = (int)(1000.0 / hz * 2.0 + 0.5);
    error_ms = (int)(1000.0 / hz * 3.0 + 0.5);
  }
  
};
};

#ifdef TESTING
#else
int main(int argc, char** argv)
{
  ros::init(argc, argv, ros::this_node::getName());

  am::AMSuper node;

  ROS_INFO_STREAM(ros::this_node::getName() << ": running...");

  ros::spin();
}
#endif
