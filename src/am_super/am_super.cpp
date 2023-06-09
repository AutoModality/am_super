#include <functional>
#include <memory>

#include <am_utils/am_ros2_utility.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/bool.hpp>

#include <am_super/baby_sitter.h>
#include <am_super/super_state.h>
#include <am_super/super_state_mediator.h>
#include <am_super/super_node_mediator.h>

#include <brain_box_msgs/msg/blink_m_command.hpp>
#include <brain_box_msgs/msg/life_cycle_state.hpp>
#include <brain_box_msgs/msg/log_control.hpp>
#include <brain_box_msgs/msg/operator_command.hpp>
#include <brain_box_msgs/msg/stamped_altimeter.hpp>
#include <brain_box_msgs/msg/super2_status.hpp>
#include <brain_box_msgs/msg/vx_state.hpp>
#include <brain_box_msgs/msg/system_state.hpp>
#include <brain_box_msgs/msg/controller_state.hpp>
#include <brain_box_msgs/msg/life_cycle_command.hpp>

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

#ifdef WIN32
  #define COLOR_NORMAL ""
  #define COLOR_RED ""
  #define COLOR_GREEN ""
  #define COLOR_YELLOW ""
#else
  #define COLOR_NORMAL "\033[0m"
  #define COLOR_RED "\033[31m"
  #define COLOR_BLUE "\033[34m"
  #define COLOR_GREEN "\033[32m"
  #define COLOR_YELLOW "\033[33m"
#endif

#undef ROS_INFO_STREAM
#undef ROS_INFO_STREAM_THROTTLE 

// Differentiate system state messages from others by blue console logs
#define ROS_INFO_STREAM(stream) RCLCPP_INFO_STREAM(am::Node::node->get_logger(), COLOR_BLUE << stream << COLOR_NORMAL)
#define ROS_INFO_STREAM_THROTTLE(duration, stream) RCLCPP_INFO_STREAM_THROTTLE(am::Node::node->get_logger(), *am::Node::node->get_clock(), (long)((duration) * 1000.0), COLOR_BLUE << stream << COLOR_NORMAL)


using namespace std;

namespace am
{
/**
 *  AM supervisor class. aggregates system state and system health and manages node lifecycle.
 *
 *  uses BabySitter instances to generate state and health for nodes that don't publish brain_box_msgs::LifeCycleState
 */
class AMSuper
{
  friend class AMSuperNode;

private:
  shared_ptr<AMLifeCycle> life_cycle_node_; 

  /** 
   * heartbeat log output period
   */
  const int LOG_THROTTLE_S = 10;

   /*
   * see constructor for details
   */
  rclcpp::Publisher<brain_box_msgs::msg::LifeCycleCommand>::SharedPtr lifecycle_pub_;
  rclcpp::Publisher<brain_box_msgs::msg::VxState>::SharedPtr vstate_summary_pub_;
  rclcpp::Publisher<brain_box_msgs::msg::SystemState>::SharedPtr system_state_pub_;
  rclcpp::Publisher<brain_box_msgs::msg::Super2Status>::SharedPtr super_status_pub_;
  rclcpp::Publisher<brain_box_msgs::msg::BlinkMCommand>::SharedPtr led_pub_;
  /** stops the flight plan when SHUTDOWN state */
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flight_plan_deactivation_pub_;
  rclcpp::Subscription<brain_box_msgs::msg::LifeCycleState>::SharedPtr node_state_sub_;
  rclcpp::Subscription<brain_box_msgs::msg::OperatorCommand>::SharedPtr operator_command_sub_;
  rclcpp::Subscription<brain_box_msgs::msg::ControllerState>::SharedPtr controller_state_sub;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_enu_sub;

  rclcpp::Subscription<brain_box_msgs::msg::LogControl>::SharedPtr log_control_sub_;
  BagLogger::BagLoggerLevel log_level_;

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
 
  typedef brain_box_msgs::msg::StampedAltimeter altimeter_bs_msg_type;
  am::BabySitter<altimeter_bs_msg_type>* altimeter_bs_;
  const std::string ALTIMETER_BS_TOPIC = "/sensor/distance/agl_lw";  // TODO: replace with system global const
  const int ALTIMETER_HZ = 20;

  const std::string NODE_BS_DJI = "dji_sdk";  // TODO: replace with system global const
  typedef sensor_msgs::msg::Joy dji_bs_msg_type;
  am::BabySitter<dji_bs_msg_type>* dji_bs_;
  const std::string DJI_BS_TOPIC = "/dji_sdk/rc";  // TODO: replace with system global const
  const int DJI_HZ = 50;

#if CUDA_FLAG
  std::shared_ptr<am::CudaUtility> gpu_info_;
#endif

public:
  AMSuper() : node_mediator_(am::Node::node, SuperNodeMediator::nodeNameStripped(am::Node::node->get_name()))
  {
    ROS_INFO_STREAM( am::Node::node->get_name());

    life_cycle_node_ = std::static_pointer_cast<AMLifeCycle>(am::Node::node);

    am::getParam<double>("node_timeout_s", node_timeout_s_, 2.0);
    ROS_INFO_STREAM( "node_timeout_s = " << node_timeout_s_);

    /*
     * create initial node list from manifest and create babysitters as needed
     */
    supervisor_.system_state = SuperState::OFF;
    // strip spaces from manifest param
    string manifest_param;
    am::getParam<std::string>("manifest", manifest_param, "");

    node_mediator_.parseManifest(supervisor_, manifest_param);

    //Add super to manifest
    node_mediator_.addSuperToManifest(supervisor_);

    // if a manifest has been specified
    if (!supervisor_.manifest.empty())
    {
    	ROS_INFO_STREAM( "configuring nodes from manifest: " << manifest_param);
      for (string& name : supervisor_.manifest)
      {
        // create a new node in the list for each name in manifest
        SuperNodeMediator::SuperNodeInfo nr = node_mediator_.initializeManifestedNode(name);
        supervisor_.nodes.insert(pair<string, SuperNodeMediator::SuperNodeInfo>(name, nr));
        ROS_INFO_STREAM( "  " << name);

        // create babysitters based on hard coded node names
        if (!name.compare(NODE_BS_ALTIMETER))
        {
          int altimeter_warn_ms, altimeter_error_ms;
          calcBSTiming(ALTIMETER_HZ, altimeter_warn_ms, altimeter_error_ms);
          altimeter_bs_ = new am::BabySitter<altimeter_bs_msg_type>(
              am::Node::node, BagLogger::instance(), name, ALTIMETER_BS_TOPIC, altimeter_warn_ms, altimeter_error_ms);
        }
        else if (!name.compare(NODE_BS_DJI))
        {
          int dji_warn_ms, dji_error_ms;
          calcBSTiming(DJI_HZ, dji_warn_ms, dji_error_ms);
          dji_bs_ = new am::BabySitter<dji_bs_msg_type>(am::Node::node, BagLogger::instance(), name, DJI_BS_TOPIC, dji_warn_ms,
                                                        dji_error_ms);
        }
      }
    }
    else
    {
      RCLCPP_WARN_STREAM(am::Node::node->get_logger(), "Manifest is empty.  No nodes will be monitored.");
    }
    
    reportSystemState();

#if CUDA_FLAG
    ROS_INFO("##########GPU Monitoring is ON##########");
    gpu_info_ = std::make_shared<am::CudaUtility>(nh_);
#endif

    /**
     * system status pub
     */
    vstate_summary_pub_ = am::Node::node->create_publisher<brain_box_msgs::msg::VxState>(am_super_topics::SUPER_STATE, 1000);
    system_state_pub_ = am::Node::node->create_publisher<brain_box_msgs::msg::SystemState>(am_topics::SYSTEM_STATE, 1000);
    /**Super
     * node lifecycle state pub. used to tell nodes to change their lifecycle state.
     */
    lifecycle_pub_ = am::Node::node->create_publisher<brain_box_msgs::msg::LifeCycleCommand>(am_super_topics::NODE_LIFECYCLE, 100);
    /**
     * led control pub
     */
    led_pub_ = am::Node::node->create_publisher<brain_box_msgs::msg::BlinkMCommand>(am::am_topics::LED_BLINK, 1000);
    /**
     * super status contains online naode list for gcs_comms
     */
    super_status_pub_ = am::Node::node->create_publisher<brain_box_msgs::msg::Super2Status>(am_super_topics::SUPER_STATUS, 1000);

    flight_plan_deactivation_pub_ = am::Node::node->create_publisher<std_msgs::msg::Bool>(am_topics::CTRL_FLIGHTPLAN_ACTIVITY_CONTROL, 1000);

    supervisor_.system_state = SuperState::BOOTING;
    supervisor_.flt_ctrl_state = SuperNodeMediator::SuperFltCtrlState::INIT;

    /**
     * amros log control
     */
    log_control_sub_ = am::Node::node->create_subscription<brain_box_msgs::msg::LogControl>(am::am_topics::CTRL_LOG_CONTROL, 10,
    		std::bind(&AMSuper::logControlCB, this, std::placeholders::_1));

    // startup bagfile - gets closed after frist log control command
    ROS_INFO_STREAM( "start logging to ST, level " << SU_LOG_LEVEL);
    BagLogger::instance()->startLogging("ST", SU_LOG_LEVEL);
    log_level_ = intToLoggerLevel (SU_LOG_LEVEL);

    // subs should always come at the end
    
    /**
     * node status via LifeCycle
     */
    node_state_sub_ = am::Node::node->create_subscription<brain_box_msgs::msg::LifeCycleState>(am_super_topics::LIFECYCLE_STATE, 100,
    		std::bind(&AMSuper::nodeStateCB, this, std::placeholders::_1));

    /**
     * commands from operator
     */
    operator_command_sub_ = am::Node::node->create_subscription<brain_box_msgs::msg::OperatorCommand>(am_super_topics::OPERATOR_COMMAND, 100,
    		std::bind(&AMSuper::operatorCommandCB, this, std::placeholders::_1));

    controller_state_sub = am::Node::node->create_subscription<brain_box_msgs::msg::ControllerState>(am_super_topics::CONTROLLER_STATE, 100,
    		std::bind(&AMSuper::controllerStateCB, this, std::placeholders::_1));

    diagnostics_sub = am::Node::node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 100,
    		std::bind(&AMSuper::diagnosticsCB, this, std::placeholders::_1));

    current_enu_sub = am::Node::node->create_subscription<nav_msgs::msg::Odometry>(am_topics::CTRL_VX_VEHICLE_CURRENTENU, 100,
    		std::bind(&AMSuper::currentENUCB, this, std::placeholders::_1));
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
  //void nodeStateCB(const rclcpp::MessageEvent<brain_box_msgs::msg::LifeCycleState const>& event)
  void nodeStateCB(const brain_box_msgs::msg::LifeCycleState::SharedPtr rmsg)
  {
    //const brain_box_msgs::LifeCycleState::ConstPtr& rmsg = event.getMessage();

    /*
     * process the message
     *
     * TODO: this should probably be more reactive. i.e. if a node sends a non-ACTIVE state
     * or a non-OK status, we should probably react to it immediately rather than wait
     * for the timeout.
     */
    processState(rmsg->node_name, (LifeCycleState)(rmsg->state), (LifeCycleStatus)(rmsg->status), rmsg->subsystem,
                 rmsg->value, rmsg->process_id, rmsg->header.stamp);

    // TODO: topic name should come from vb_util_lib::topics.h
    LOG_MSG(am_super_topics::LIFECYCLE_STATE, *rmsg, SU_LOG_LEVEL);
  }

  //void controllerStateCB(const ros::MessageEvent<brain_box_msgs::ControllerState const>& event)
  void controllerStateCB(const brain_box_msgs::msg::ControllerState::SharedPtr rmsg)
  {
    //const brain_box_msgs::ControllerState::ConstPtr& rmsg = event.getMessage();

    RCLCPP_INFO(am::Node::node->get_logger(), "Received Controller State: %s sent %i", rmsg->node_name.c_str(), rmsg->state);
    node_mediator_.setControllerState(supervisor_, (ControllerState)rmsg->state);
  }

  //void operatorCommandCB(const ros::MessageEvent<brain_box_msgs::OperatorCommand const>& event)
  void operatorCommandCB(const brain_box_msgs::msg::OperatorCommand::SharedPtr rmsg)
  {
    //const brain_box_msgs::OperatorCommand::ConstPtr& rmsg = event.getMessage();
    
	RCLCPP_INFO(am::Node::node->get_logger(), "Received Operator Command: %s sent '%i'",rmsg->node_name.c_str(),rmsg->command );
    
    node_mediator_.setOperatorCommand(supervisor_, (OperatorCommand)rmsg->command);
    // TODO: topic name should come from vb_util_lib::topics.
    LOG_MSG("/operator/command", *rmsg,  SU_LOG_LEVEL);
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
                    const rclcpp::Time& last_contact)
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
        ROS_INFO_STREAM( "manifested node '" << node_name << "' came online [PGPG]");
        nr.online = true;
        nodes_changed = true;
      }
      if (nr.state != state)
      {
    	ROS_INFO_STREAM( node_name << " changed state to = " << life_cycle_mediator_.stateToString(state) << " [38S8]");
        nr.state = state;
        nodes_changed = true;
      }
      if (nr.status != status)
      {
    	ROS_INFO_STREAM( node_name << " changed status to = " << life_cycle_mediator_.statusToString(status) << " [09SI]");
        nr.status = status;
        nodes_changed = true;
        if(nr.manifested && nr.status == LifeCycleStatus::ERROR)
        {
          supervisor_.status_error = true;
          ROS_INFO_STREAM( "Manifested node " << nr.name << " changed status to ERROR. Shutting down nodes... [JHRE]");
          stopFlightPlan();
        }
      }
      if (nr.pid != pid)
      {
        //process id = 0 observed to be a node coming online. -1 appears to be offline
        if(pid == 0)
        {
        	ROS_INFO_STREAM( node_name << " process is alive [UIRE]");
        }
        else
        {
        	ROS_INFO_STREAM( node_name << " changed process id from: " << nr.pid << " to: " <<  pid << " [WNEW]");
        }
        nr.pid = pid;
        nodes_changed = true;
      }
      nr.last_contact = last_contact;
    }
    else
    {
      // if we get here, the node is not in the manifest and we've never heard from it before
      RCLCPP_WARN_STREAM(am::Node::node->get_logger(), "unknown node " << node_name << " came online. state: " << life_cycle_mediator_.stateToString(state)
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
        RCLCPP_INFO_STREAM_THROTTLE(am::Node::node->get_logger(), *am::Node::node->get_clock(), 1.0, "flight status: " << value);
        checkForSystemStateTransition();
      }
    }
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
    ROS_INFO_STREAM_THROTTLE(LOG_THROTTLE_S, ss.str());
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
    brain_box_msgs::msg::LifeCycleCommand msg;
    msg.node_name = node_name;
    msg.command = (brain_box_msgs::msg::LifeCycleCommand::_command_type)command;
    lifecycle_pub_->publish(msg);
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
  bool allManifestedNodesCheck(std::function<bool(SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check)
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


  /** Send signal to flight controller that flight is over. */
  void stopFlightPlan()
  {
    std_msgs::msg::Bool msg;
    msg.data = false; //false means deactivate
    flight_plan_deactivation_pub_->publish(msg);
    ROS_ERROR_STREAM( "Sending flight plan kill command.");
  }

  /**
   * check for state transition based upon current state and values of member fields.
   * Will call to modify the system state if transition is necessary. Will also call
   * lifecycle command if indicated to do so based on the state.
   * See SuperNodeMediator::TransitionInstructions
   */
  void checkForSystemStateTransition()
  {
    if(life_cycle_node_->getState() == LifeCycleState::INACTIVE && supervisor_.system_state == SuperState::READY) //if super lifecycle is currently inactive
    {
      ROS_INFO_STREAM("Automatically activating am_super");
      sendLifeCycleCommand(node_mediator_.getNodeName(), LifeCycleCommand::ACTIVATE); 
    }
    else
    {
      
      SuperNodeMediator::TransitionInstructions transition_instructions = node_mediator_.transitionReady(supervisor_);

      if (transition_instructions.ready_for_transition)
      {
        setSystemState(transition_instructions.new_state);
      }
      if (transition_instructions.resend_life_cycle_command)
      {
        LifeCycleCommand command = transition_instructions.life_cycle_command;
        std::string failed_nodes_string = boost::algorithm::join(transition_instructions.failed_nodes, ", ");
        std::string failed_nodes_reasons_string = boost::algorithm::join(transition_instructions.failed_nodes_reasons, ", ");
        ROS_INFO_STREAM_THROTTLE(5,state_mediator_.stateToString(supervisor_.system_state)
                        << ": sending " << life_cycle_mediator_.commandToString(command) << " to "
                        << failed_nodes_string << " because " << failed_nodes_reasons_string);

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
    bool legal = true;
    if(!node_mediator_.forceTransition(state))
      legal = state_mediator_.allowsTransition(supervisor_.system_state, state);

    if (!legal)
    {
      ROS_ERROR_STREAM("illegal state transition from " << state_mediator_.stateToString(supervisor_.system_state)
                                                        << " to " << state_mediator_.stateToString(state));
    }
    else
    {

      // persist given state as the new current state
      supervisor_.system_state = state;
      
      reportSystemState();

      sendLEDMessage();

      brain_box_msgs::msg::VxState state_msg;
      state_msg.state = (uint8_t)supervisor_.system_state;
      vstate_summary_pub_->publish(state_msg);
    }
  }


  /** load the platform configurations from the launch file and populate the variants provided.
   */
  void configurePlatformRequirements(SuperNodeMediator::PlatformVariant &required_platform,
                                     SuperNodeMediator::PlatformVariant &actual_platform)
  {
    //actual platform is required or we fail 
    std::string not_provided = "none";
    std::string actual_platform_param;
    life_cycle_node_->param("platform.actual",actual_platform_param,not_provided);
    if(actual_platform_param == not_provided)
    {
      life_cycle_node_->errorTerminal("param `am_super.platform.actual` must provide the platform running","NNS9");
      return;
    }
    node_mediator_.platformConfigToVariant(actual_platform_param,actual_platform);

    //compare actual platform to required platform, if provided
    std::string required_platform_param;
    std::string platform_app_required_param;
    life_cycle_node_->param("platform/required",required_platform_param,not_provided);
    life_cycle_node_->param("platform/app/required",platform_app_required_param,not_provided);
    if(required_platform_param != not_provided)
    {
      node_mediator_.platformConfigToVariant(required_platform_param,required_platform);
    }
    else if(platform_app_required_param != not_provided)
    {
      required_platform.app = platform_app_required_param;
    }
    else
    {
      ROS_WARN("platform requirements not set");
    }

  }
  /**
   * send led color message based on raw values
   */
  void sendLEDMessage(int r, int g, int b, float period = 0.0)
  {
    brain_box_msgs::msg::BlinkMCommand led_msg;
    led_msg.rgb.red = r;
    led_msg.rgb.green = g;
    led_msg.rgb.blue = b;
    led_msg.blink_rate = period;
    led_pub_->publish(led_msg);
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
  
  void diagnosticsCB(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
      LOG_MSG("/diagnostics", *msg, SU_LOG_LEVEL);
  }

  void currentENUCB(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
      LOG_MSG(am_topics::CTRL_VX_VEHICLE_CURRENTENU, *msg, SU_LOG_LEVEL);
  }

  BagLogger::BagLoggerLevel intToLoggerLevel(int level)
  {
    switch (level)
    {
      case 1:
        return BagLogger::NORM;
        break;
      case 2:
        return BagLogger::FINE;
        break;
      case 3:
        return BagLogger::EXTRA;
        break;
      case 4:
        return BagLogger::ALL;
        break;
      default:
        return BagLogger::OFF;
        break;
    }
  }

  void logControlCB(const brain_box_msgs::msg::LogControl::SharedPtr msg)
   {
     if (msg->enable)
     {
       ROS_INFO_STREAM("stop logging");
       BagLogger::instance()->stopLogging();

       ROS_INFO_STREAM("start logging to SU, level " << SU_LOG_LEVEL);
       BagLogger::instance()->startLogging("SU", SU_LOG_LEVEL);
       log_level_ = intToLoggerLevel (SU_LOG_LEVEL);
     }
   }

};

class AMSuperNode : public AMLifeCycle
{
private:
  shared_ptr<AMSuper> am_super_;

public:
  AMSuperNode(const std::string & node_name) : AMLifeCycle(node_name)
  {
  }

  ~AMSuperNode()
  {
  }

  void setAMSuper(shared_ptr<AMSuper> am_super)
  {
    am_super_= am_super;
  }

/**
   * Verify the basic requirements are being met:
   * - platform required matches actual platform
   */  
  void onConfigure() override
  {
    if(am_super_ == nullptr)
    {
      AMLifeCycle::onConfigure();
      return;
    }

    SuperNodeMediator::PlatformVariant required_platform;
    SuperNodeMediator::PlatformVariant actual_platform;
    am_super_->configurePlatformRequirements(required_platform, actual_platform);
    ROS_WARN_STREAM("required" << required_platform.maker);
    ROS_WARN_STREAM("actual" << actual_platform.maker);
    if(!am_super_->node_mediator_.isCorrectPlatform(required_platform,actual_platform))
    {
      std::stringstream message;
      message << "Platform required: `" 
              << am_super_->node_mediator_.platformVariantToConfig(required_platform)
              << "` actual: `" 
              << am_super_->node_mediator_.platformVariantToConfig(actual_platform)
              ;
      errorTerminal(message.str(),"NSK2"); //force failure since this is not recoverable
    }
    else
    {
      AMLifeCycle::onConfigure();
    }
  }

 /**
   * called once per second.
   *
   * times out nodes that haven't been heard from recently. reports on status to bag and trace logs.
   */
  void heartbeatCB() override
  {
    if(am_super_ == nullptr)
    {
      AMLifeCycle::heartbeatCB();
      return;
    }

#if CUDA_FLAG
    gpu_info_->display();
#endif

    //publish deprecated topic
    {
      brain_box_msgs::msg::VxState state_msg;
      state_msg.state = (uint8_t)am_super_->supervisor_.system_state;
      am_super_->vstate_summary_pub_->publish(state_msg);
    }

    //publish the system state
    {
      brain_box_msgs::msg::SystemState system_state_msg;
      system_state_msg.state = (uint8_t)am_super_->supervisor_.system_state;
      system_state_msg.state_string = am_super_->state_mediator_.stateToString(am_super_->supervisor_.system_state);
      am_super_->system_state_pub_->publish(system_state_msg);
    }

    // cycle thru all the nodes in the list to look for a timeout
    rclcpp::Time now = am::Node::node->now();
    map<string, SuperNodeMediator::SuperNodeInfo>::iterator it;
    for (it = am_super_->supervisor_.nodes.begin(); it != am_super_->supervisor_.nodes.end(); it++)
    {
      SuperNodeMediator::SuperNodeInfo& nr = (*it).second;
      if (nr.online)
      {
        rclcpp::Duration time_since_contact = (now - nr.last_contact);
        rclcpp::Duration timeout_dur(am::toDuration(am_super_->node_timeout_s_));
        if (time_since_contact > timeout_dur)
        {
          nr.online = false;
          ROS_ERROR_STREAM("node timed out:" << nr.name);
          am_super_->reportSystemState();
        }
      }
    }

    // check for state transition due to timeouts or anything else that changed since last heartbeat
    am_super_->checkForSystemStateTransition();

    int num_manifest_nodes_online = am_super_->node_mediator_.manifestedNodesOnlineCount(am_super_->supervisor_);
    // publish and bag log super status message
    brain_box_msgs::msg::Super2Status status_msg;
    status_msg.man = am_super_->supervisor_.manifest.size();
    status_msg.man_run = num_manifest_nodes_online;
    status_msg.run = am_super_->node_mediator_.nodesOnlineCount(am_super_->supervisor_);

    for (it = am_super_->supervisor_.nodes.begin(); it != am_super_->supervisor_.nodes.end(); it++)
    {
      SuperNodeMediator::SuperNodeInfo& nr = (*it).second;
      status_msg.nodes.push_back(nr.name);
    }
    LOG_MSG("/status/super", status_msg, 1);
    if (am_super_->super_status_pub_->get_subscription_count() > 0)
    {
      am_super_->super_status_pub_->publish(status_msg);
    }

    // report current status to trace log
    std::stringstream ss;
    am_super_->genSystemState(ss);

    if (am_super_->supervisor_.manifest.size() != num_manifest_nodes_online)
    {
      // if all manifested nodes aren't running, report as error
      ROS_ERROR_STREAM(ss.str());
      ROS_ERROR_STREAM("not online: " << am_super_->node_mediator_.manifestedNodesNotOnlineNamesList(am_super_->supervisor_));
    }
    else
    {
      // if all manifested nodes are running, report as info
      ROS_INFO_STREAM_THROTTLE(am_super_->LOG_THROTTLE_S, ss.str());
    }

    // log stats
    fstream newfile;
    newfile.open("/sys/bus/i2c/devices/7-0040/iio_device/in_power0_input",ios::in); //open a file to perform read operation using file object
    if (newfile.is_open())
    {   //checking whether the file is open
       string tp;
       getline(newfile, tp);
       std_msgs::msg::Int16 msg;
       msg.data = std::stoi(tp);
       LOG_MSG("/watts", msg, am_super_->SU_LOG_LEVEL);
       newfile.close(); //close the file object.
    }

    AMLifeCycle::heartbeatCB();
  }

};

}; // namespace

// #ifdef TESTING
// #else

shared_ptr<am::AMLifeCycle> am::Node::node;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  shared_ptr<am::AMSuperNode> am_super_node = make_shared<am::AMSuperNode>("am_super");
  am::Node::node = am_super_node;

  std::shared_ptr<am::AMSuper> am_super = make_shared<am::AMSuper>();
  am_super_node->setAMSuper(am_super);

  ROS_INFO_STREAM(am::Node::node->get_name() << ": running...");

  rclcpp::spin(am::Node::node);

  rclcpp::shutdown();

  return 0;
}
// #endif
