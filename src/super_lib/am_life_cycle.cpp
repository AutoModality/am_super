#include <super_lib/am_life_cycle.h>
#include <brain_box_msgs/LifeCycleState.h>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>




namespace am
{
  

// static constexpr std::string AMLifeCycle::STATE_INVALID_STRING;
// static constexpr std::string AMLifeCycle::STATE_UNCONFIGURED_STRING;

AMLifeCycle::AMLifeCycle() : nh_("~")
{
  std::string init_state_str;
  ros::param::param<std::string>("~init_state", init_state_str, "ACTIVE");
  ROS_INFO_STREAM("init_state = " << init_state_str);
  LifeCycleState init_state;
  if (stringToState(init_state_str, init_state))
  {
    state_ = init_state;
  }
  else
  {
    state_ = LifeCycleState::ACTIVE;
  }
  status_ = LifeCycleStatus::OK;
  state_pub_ = nh_.advertise<brain_box_msgs::LifeCycleState>("/node_state", 100);

  updater_.setHardwareID("none");
  updater_.broadcast(0, "Initializing node");
  updater_.add("diagnostics", this, &AMLifeCycle::addStatistics);
  updater_.force_update();

  // strip leading '/' if it is there
  // TODO: this might always be there so just strip it without checking?
  if (ros::this_node::getName().at(0) == '/')
  {
    node_name_ = ros::this_node::getName().substr(1);
  }
  else
  {
    node_name_ = ros::this_node::getName();
  }

  // subs should always come at the end
  /**
   * node status via LifeCycle
   */
  lifecycle_sub_ = nh_.subscribe("/node_lifecycle", 100, &AMLifeCycle::lifecycleCB, this);

  heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0), &AMLifeCycle::heartbeatCB, this);
}

AMLifeCycle::~AMLifeCycle()
{
}

void AMLifeCycle::lifecycleCB(const brain_box_msgs::LifeCycleCommand::ConstPtr msg)
{
  ROS_DEBUG_STREAM_THROTTLE(1.0, commandToString((LifeCycleCommand)msg->command));

  if (!msg->node_name.compare(AMLifeCycle::BROADCAST_NODE_NAME) || !msg->node_name.compare(node_name_))
  {
    // if this message is for us
    switch ((LifeCycleCommand)msg->command)
    {
      case LifeCycleCommand::ACTIVATE:
        transition("activate", LifeCycleState::INACTIVE, LifeCycleState::ACTIVATING, LifeCycleState::ACTIVE,
                   std::bind(&AMLifeCycle::onActivate, this));
        break;
      case LifeCycleCommand::CLEANUP:
        transition("cleanup", LifeCycleState::INACTIVE, LifeCycleState::CLEANING_UP, LifeCycleState::UNCONFIGURED,
                   std::bind(&AMLifeCycle::onCleanup, this));
        break;
      case LifeCycleCommand::CONFIGURE:
        transition("configure", LifeCycleState::UNCONFIGURED, LifeCycleState::CONFIGURING, LifeCycleState::INACTIVE,
                   std::bind(&AMLifeCycle::onConfigure, this));
        break;
      case LifeCycleCommand::CREATE:
        ROS_WARN_STREAM("illegal command " << commandToString(LifeCycleCommand::CREATE));
        break;
      case LifeCycleCommand::DEACTIVATE:
        transition("deactivate", LifeCycleState::ACTIVE, LifeCycleState::DEACTIVATING, LifeCycleState::INACTIVE,
                   std::bind(&AMLifeCycle::onDeactivate, this));
        break;
      case LifeCycleCommand::DESTROY:
        destroy();
        break;
      case LifeCycleCommand::SHUTDOWN:
        shutdown();
        break;
    }
  }
}

void AMLifeCycle::transition(std::string transition_name, LifeCycleState initial_state, LifeCycleState transition_state,
                             LifeCycleState final_state, std::function<void(void)> on_function)
{
  if (state_ == initial_state)
  {
    ROS_INFO_STREAM(transition_name << ", current state: " << stateToString(state_));
    setState(transition_state);
    on_function();
  }
  else if (state_ == transition_state || state_ == final_state)
  {
    ROS_DEBUG_STREAM("ignoring redundant " << transition_name);
  }
  else
  {
    ROS_WARN_STREAM("received illegal activate in state " << stateToString(state_));
  }
}

void AMLifeCycle::doTransition(std::string transition_name, bool success, LifeCycleState success_state,
                               LifeCycleState failure_state)
{
  logState();
  if (success)
  {
    ROS_INFO_STREAM(transition_name << " succeeded");
    setState(success_state);
  }
  else
  {
    ROS_INFO_STREAM(transition_name << " failed");
    setState(failure_state);
  }
}

void AMLifeCycle::onActivate()
{
  logState();
  doActivate(true);
}

void AMLifeCycle::doActivate(bool success)
{
  doTransition("activation", success, LifeCycleState::ACTIVE, LifeCycleState::INACTIVE);
}

void AMLifeCycle::onCleanup()
{
  logState();
  doCleanup(true);
}

void AMLifeCycle::doCleanup(bool success)
{
  doTransition("cleanup", success, LifeCycleState::UNCONFIGURED, LifeCycleState::INACTIVE);
  logState();
}

void AMLifeCycle::onConfigure()
{
  doConfigure(true);
}

void AMLifeCycle::doConfigure(bool success)
{
  doTransition("configuration", success, LifeCycleState::INACTIVE, LifeCycleState::UNCONFIGURED);
}

void AMLifeCycle::onDeactivate()
{
  logState();
  doDeactivate(true);
}

void AMLifeCycle::logState()
{
    ROS_INFO_STREAM("LifeCycle: " << stateToString(state_));
}

void AMLifeCycle::doDeactivate(bool success)
{
  doTransition("deactivation", success, LifeCycleState::INACTIVE, LifeCycleState::ACTIVE);
}

void AMLifeCycle::destroy()
{
  if (state_ != LifeCycleState::FINALIZED)
  {
    ROS_INFO_STREAM("received illegal activate in state " << stateToString(state_));
  }
  else if (state_ == LifeCycleState::SHUTTING_DOWN || state_ == LifeCycleState::FINALIZED)
  {
    ROS_INFO_STREAM("current state: " << stateToString(state_));
    onDestroy();
  }
}

void AMLifeCycle::onDestroy()
{
  logState();
  doDestroy(true);
}

void AMLifeCycle::doDestroy(bool success)
{
  logState();
  // TODO: how do we call node destructor and exit main()? raise some type of ROS error?
}

void AMLifeCycle::error()
{
  if (state_ == LifeCycleState::ERROR_PROCESSING || state_ == LifeCycleState::FINALIZED ||
      state_ == LifeCycleState::UNCONFIGURED)
  {
    ROS_DEBUG_STREAM("ignoring redundant error");
  }
  else
  {
    ROS_INFO_STREAM("current state: " << stateToString(state_));
    setState(LifeCycleState::ERROR_PROCESSING);
    onError();
  }
}

void AMLifeCycle::onError()
{
  logState();
  doError(true);
}

void AMLifeCycle::doError(bool success)
{
  logState();
  if (success)
  {
    state_ = LifeCycleState::UNCONFIGURED;
  }
  else
  {
    state_ = LifeCycleState::FINALIZED;
  }
  sendNodeUpdate();
}

void AMLifeCycle::shutdown()
{
  if (state_ == LifeCycleState::UNCONFIGURED || state_ == LifeCycleState::INACTIVE || state_ == LifeCycleState::ACTIVE)
  {
    ROS_INFO_STREAM("current state: " << stateToString(state_));
    setState(LifeCycleState::SHUTTING_DOWN);
    onShutdown();
  }
  else if (state_ == LifeCycleState::SHUTTING_DOWN || state_ == LifeCycleState::FINALIZED)
  {
    ROS_DEBUG_STREAM("ignoring redundant shutdown");
  }
  else
  {
    ROS_INFO_STREAM("received illegal activate in state " << stateToString(state_));
  }
}

void AMLifeCycle::onShutdown()
{
  logState();
  doShutdown(true);
}

void AMLifeCycle::doShutdown(bool success)
{
  logState();
  setState(LifeCycleState::FINALIZED);
}

void AMLifeCycle::addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw)
{
  stats_list_.addStatistics(dsw);
  LifeCycleStatus status = stats_list_.process(warn_throttle_s_, error_throttle_s_);
  setStatus(status);
  dsw.summary((uint8_t)status, "update");
}

void AMLifeCycle::sendNodeUpdate()
{
  brain_box_msgs::LifeCycleState msg;
  msg.node_name = ros::this_node::getName();
  msg.process_id = 0;
  msg.state = (uint8_t)state_;
  msg.status = (uint8_t)status_;
  msg.subsystem = "";
  msg.value = "";
  state_pub_.publish(msg);
}
void AMLifeCycle::heartbeatCB(const ros::TimerEvent& event)
{
  updater_.force_update();

  std::stringstream ss;
  ss << AMLifeCycle::stateToString(state_) << "," << AMLifeCycle::statusToString(status_) << ","
     << stats_list_.getStatsStrShort();

  double throttle;
  switch (status_)
  {
    case LifeCycleStatus::OK:
      throttle = ok_throttle_s_;
      break;
    case LifeCycleStatus::WARN:
      throttle = warn_throttle_s_;
      break;
    case LifeCycleStatus::ERROR:
      throttle = error_throttle_s_;
      break;
  }
  ROS_INFO_STREAM_THROTTLE(ok_throttle_s_, "LifeCycle heartbeat: " << ss.str());

  stats_list_.reset();

  sendNodeUpdate();
}

typedef boost::bimap<std::string_view, am::LifeCycleState> str_state_bimap;
const str_state_bimap str_state_bimap_ = boost::assign::list_of< str_state_bimap::relation > 
  (AMLifeCycle::STATE_INVALID_STRING, LifeCycleState::INVALID)
  (AMLifeCycle::STATE_UNCONFIGURED_STRING, LifeCycleState::UNCONFIGURED)
  (AMLifeCycle::STATE_INACTIVE_STRING, LifeCycleState::INACTIVE)
  (AMLifeCycle::STATE_ACTIVE_STRING, LifeCycleState::ACTIVE)
  (AMLifeCycle::STATE_FINALIZED_STRING, LifeCycleState::FINALIZED)
  (AMLifeCycle::STATE_CONFIGURING_STRING, LifeCycleState::CONFIGURING)
  (AMLifeCycle::STATE_CLEANING_UP_STRING, LifeCycleState::CLEANING_UP)
  (AMLifeCycle::STATE_ACTIVATING_STRING, LifeCycleState::ACTIVATING)
  (AMLifeCycle::STATE_DEACTIVATING_STRING, LifeCycleState::DEACTIVATING)
  (AMLifeCycle::STATE_ERROR_PROCESSING_STRING, LifeCycleState::ERROR_PROCESSING)
  (AMLifeCycle::STATE_SHUTTING_DOWN, LifeCycleState::SHUTTING_DOWN);

const std::string_view& AMLifeCycle::stateToString(LifeCycleState state)
{
    if(str_state_bimap_.right.count(state))
    {
      return str_state_bimap_.right.at(state);
    }
    return STATE_INVALID_STRING;
}

bool AMLifeCycle::stringToState(std::string& state_str, LifeCycleState& state)
{
  if(str_state_bimap_.left.count(state_str))
  {
    state = str_state_bimap_.left.at(state_str);
    return true;
  }
  return false;
}

typedef boost::bimap<std::string_view, am::LifeCycleStatus> str_status_bimap;
const str_status_bimap str_status_bimap_ = boost::assign::list_of< str_status_bimap::relation > 
  (AMLifeCycle::STATUS_OK_STRING, LifeCycleStatus::OK)
  (AMLifeCycle::STATUS_WARN_STRING, LifeCycleStatus::WARN)
  (AMLifeCycle::STATUS_ERROR_STRING, LifeCycleStatus::ERROR);

const std::string_view& AMLifeCycle::statusToString(LifeCycleStatus status)
{
  if(str_status_bimap_.right.count(status))
    {
      return str_status_bimap_.right.at(status);
    }
    return AMLifeCycle::EMPTY_STRING;
}

bool AMLifeCycle::stringToStatus(std::string& status_str, LifeCycleStatus& status)
{
  if(str_status_bimap_.left.count(status_str))
  {
    status = str_status_bimap_.left.at(status_str);
    return true;
  }
  return false;
}

typedef boost::bimap<std::string_view, am::LifeCycleCommand> str_command_bimap;
const str_command_bimap str_command_bimap_ = boost::assign::list_of< str_command_bimap::relation > 
  (AMLifeCycle::COMMAND_ACTIVATE_STRING, LifeCycleCommand::ACTIVATE)
  (AMLifeCycle::COMMAND_CLEANUP_STRING, LifeCycleCommand::CLEANUP)
  (AMLifeCycle::COMMAND_CONFIGURE_STRING, LifeCycleCommand::CONFIGURE)
  (AMLifeCycle::COMMAND_CREATE_STRING, LifeCycleCommand::CREATE)
  (AMLifeCycle::COMMAND_DEACTIVATE_STRING, LifeCycleCommand::DEACTIVATE)
  (AMLifeCycle::COMMAND_DESTROY_STRING, LifeCycleCommand::DESTROY)
  (AMLifeCycle::COMMAND_SHUTDOWN_STRING, LifeCycleCommand::SHUTDOWN);

const std::string_view& AMLifeCycle::commandToString(LifeCycleCommand command)
{
  if(str_command_bimap_.right.count(command))
    {
      return str_command_bimap_.right.at(command);
    }
    return EMPTY_STRING;
}

bool AMLifeCycle::stringToCommand(std::string& command_str, LifeCycleCommand& command)
{
  if(str_command_bimap_.left.count(command_str))
  {
    command = str_command_bimap_.left.at(command_str);
    return true;
  }
  return false;
}

LifeCycleState AMLifeCycle::getState() const
{
  return state_;
}

void AMLifeCycle::setState(const LifeCycleState state)
{
  if (state < LifeCycleState::LAST_STATE)
  {
    ROS_INFO_STREAM("changing state from " << stateToString(state_) << " to " << stateToString(state));
    state_ = state;
    sendNodeUpdate();
  }
  else
  {
    ROS_ERROR_STREAM("illegal state: " << (int)state);
  }
}

const std::vector<LifeCycleCommand> AMLifeCycle::getLifeCycleCommands()
{
  std::vector<LifeCycleCommand> all;
  for (int enumIndex = (int)LifeCycleCommand::CREATE; enumIndex <= (int)LifeCycleCommand::LAST_COMMAND; enumIndex++)
  {
    LifeCycleCommand command = static_cast<LifeCycleCommand>(enumIndex);
    all.push_back(command);
  }
  return all;
}

const std::vector<LifeCycleState> AMLifeCycle::getLifeCycleStates()
{
  std::vector<LifeCycleState> all;
  for (int enumIndex = (int)LifeCycleState::INVALID; enumIndex <= (int)LifeCycleState::LAST_STATE; enumIndex++)
  {
    LifeCycleState state = static_cast<LifeCycleState>(enumIndex);
    all.push_back(state);
  }
  return all;
}

const std::vector<LifeCycleStatus> AMLifeCycle::getLifeCycleStatuses()
{
  std::vector<LifeCycleStatus> all;
  for (int enumIndex = (int)LifeCycleStatus::OK; enumIndex <= (int)LifeCycleStatus::LAST_STATUS; enumIndex++)
  {
    LifeCycleStatus Status = static_cast<LifeCycleStatus>(enumIndex);
    all.push_back(Status);
  }
  return all;
}

LifeCycleStatus AMLifeCycle::getStatus() const
{
  return status_;
}

bool AMLifeCycle::setStatus(const LifeCycleStatus status)
{
  if (status == LifeCycleStatus::LAST_STATUS)
  {
      return false;
  }
  status_ = status;
  return true;
}

void AMLifeCycle::setThrottleS(const double throttleS)
{
  if (throttleS == 0.0)
  {
    ok_throttle_s_ = DEFAULT_OK_THROTTLE_S;
    warn_throttle_s_ = DEFAULT_WARN_THROTTLE_S;
    error_throttle_s_ = DEFAULT_ERROR_THROTTLE_S;
  }
  else
  {
    ok_throttle_s_ = throttleS;
    warn_throttle_s_ = throttleS;
    error_throttle_s_ = throttleS;
  }
}



};

