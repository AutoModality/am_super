#include <super_lib/am_life_cycle.h>
#include <brain_box_msgs/LifeCycleState.h>

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
    ROS_INFO_STREAM("received illegal activate in state " << stateToString(state_));
  }
}

void AMLifeCycle::doTransition(std::string transition_name, bool success, LifeCycleState success_state,
                               LifeCycleState failure_state)
{
  ROS_INFO_STREAM(stateToString(state_));
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
  ROS_INFO_STREAM(stateToString(state_));
  doActivate(true);
}

void AMLifeCycle::doActivate(bool success)
{
  doTransition("activation", success, LifeCycleState::ACTIVE, LifeCycleState::INACTIVE);
}

void AMLifeCycle::onCleanup()
{
  ROS_INFO_STREAM(stateToString(state_));
  doCleanup(true);
}

void AMLifeCycle::doCleanup(bool success)
{
  doTransition("cleanup", success, LifeCycleState::UNCONFIGURED, LifeCycleState::INACTIVE);
  ROS_INFO_STREAM(stateToString(state_));
}

void AMLifeCycle::onConfigure()
{
  ROS_INFO_STREAM(stateToString(state_));
  doConfigure(true);
}

void AMLifeCycle::doConfigure(bool success)
{
  doTransition("configuration", success, LifeCycleState::INACTIVE, LifeCycleState::UNCONFIGURED);
}

void AMLifeCycle::onDeactivate()
{
  ROS_INFO_STREAM(stateToString(state_));
  doDeactivate(true);
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
  ROS_INFO_STREAM(stateToString(state_));
  doDestroy(true);
}

void AMLifeCycle::doDestroy(bool success)
{
  ROS_INFO_STREAM(stateToString(state_));
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
  ROS_INFO_STREAM(stateToString(state_));
  doError(true);
}

void AMLifeCycle::doError(bool success)
{
  ROS_INFO_STREAM(stateToString(state_));
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
  ROS_INFO_STREAM(stateToString(state_));
  doShutdown(true);
}

void AMLifeCycle::doShutdown(bool success)
{
  ROS_INFO_STREAM(stateToString(state_));
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

  switch (status_)
  {
    case LifeCycleStatus::OK:
      ROS_INFO_STREAM_THROTTLE(ok_throttle_s_, ss.str());
      break;
    case LifeCycleStatus::WARN:
      ROS_WARN_STREAM_THROTTLE(warn_throttle_s_, ss.str());
      break;
    case LifeCycleStatus::ERROR:
      ROS_ERROR_STREAM_THROTTLE(error_throttle_s_, ss.str());
      break;
  }

  stats_list_.reset();

  sendNodeUpdate();
}

const std::string_view& AMLifeCycle::stateToString(LifeCycleState state)
{
  switch (state)
  {
    case LifeCycleState::UNCONFIGURED:
      return STATE_UNCONFIGURED_STRING;
    case LifeCycleState::INACTIVE:
      return STATE_INACTIVE_STRING;
    case LifeCycleState::ACTIVE:
      return STATE_ACTIVE_STRING;
    case LifeCycleState::FINALIZED:
      return STATE_FINALIZED_STRING;
    case LifeCycleState::CONFIGURING:
      return STATE_CONFIGURING_STRING;
    case LifeCycleState::CLEANING_UP:
      return STATE_CLEANING_UP_STRING;
    case LifeCycleState::ACTIVATING:
      return STATE_ACTIVATING_STRING;
    case LifeCycleState::DEACTIVATING:
      return STATE_DEACTIVATING_STRING;
    case LifeCycleState::ERROR_PROCESSING:
      return STATE_ERROR_PROCESSING_STRING;
    case LifeCycleState::SHUTTING_DOWN:
      return STATE_SHUTTING_DOWN;
    case LifeCycleState::INVALID:
    default:
      return STATE_INVALID_STRING;
  }
}

bool AMLifeCycle::stringToState(std::string& state_str, LifeCycleState& state)
{
  if (!state_str.compare(STATE_UNCONFIGURED_STRING))
  {
    state = LifeCycleState::UNCONFIGURED;
  }
  else if (!state_str.compare(STATE_INACTIVE_STRING))
  {
    state = LifeCycleState::INACTIVE;
  }
  else if (!state_str.compare(STATE_ACTIVE_STRING))
  {
    state = LifeCycleState::ACTIVE;
  }
  else if (!state_str.compare(STATE_FINALIZED_STRING))
  {
    state = LifeCycleState::FINALIZED;
  }
  else if (!state_str.compare(STATE_CONFIGURING_STRING))
  {
    state = LifeCycleState::CONFIGURING;
  }
  else if (!state_str.compare(STATE_CLEANING_UP_STRING))
  {
    state = LifeCycleState::CLEANING_UP;
  }
  else if (!state_str.compare(STATE_ACTIVATING_STRING))
  {
    state = LifeCycleState::ACTIVATING;
  }
  else if (!state_str.compare(STATE_DEACTIVATING_STRING))
  {
    state = LifeCycleState::DEACTIVATING;
  }
  else if (!state_str.compare(STATE_ERROR_PROCESSING_STRING))
  {
    state = LifeCycleState::ERROR_PROCESSING;
  }
  else if (!state_str.compare(STATE_INVALID_STRING))
  {
    state = LifeCycleState::INVALID;
  }
  else if (!state_str.compare(STATE_SHUTTING_DOWN))
  {
    state = LifeCycleState::SHUTTING_DOWN;
  }
  else
  {
    return false;
  }
  return true;
}

const std::string_view& AMLifeCycle::statusToString(LifeCycleStatus state)
{
  switch (state)
  {
    case LifeCycleStatus::OK:
      return STATUS_OK_STRING;
    case LifeCycleStatus::WARN:
      return STATUS_WARN_STRING;
    case LifeCycleStatus::ERROR:
      return STATUS_ERROR_STRING;
    default:
      return EMPTY_STRING;
  }
}

bool AMLifeCycle::stringToStatus(std::string& status_str, LifeCycleStatus& status)
{
  if (!status_str.compare(STATUS_OK_STRING))
  {
    status = LifeCycleStatus::OK;
  }
  else if (!status_str.compare(STATUS_WARN_STRING))
  {
    status = LifeCycleStatus::WARN;
  }
  else if (!status_str.compare(STATUS_ERROR_STRING))
  {
    status = LifeCycleStatus::ERROR;
  }
  else
  {
    return false;
  }
  return true;
}

const std::string_view& AMLifeCycle::commandToString(LifeCycleCommand state)
{
  switch (state)
  {
    case LifeCycleCommand::CREATE:
      return COMMAND_CREATE_STRING;
    case LifeCycleCommand::CONFIGURE:
      return COMMAND_CONFIGURE_STRING;
    case LifeCycleCommand::CLEANUP:
      return COMMAND_CLEANUP_STRING;
    case LifeCycleCommand::ACTIVATE:
      return COMMAND_ACTIVATE_STRING;
    case LifeCycleCommand::DEACTIVATE:
      return COMMAND_DEACTIVATE_STRING;
    case LifeCycleCommand::SHUTDOWN:
      return COMMAND_SHUTDOWN_STRING;
    case LifeCycleCommand::DESTROY:
      return COMMAND_DESTROY_STRING;
    default:
      return EMPTY_STRING;
  }
}

bool AMLifeCycle::stringToCommand(std::string& status_str, LifeCycleCommand& status)
{
  if (!status_str.compare(COMMAND_CREATE_STRING))
  {
    status = LifeCycleCommand::CREATE;
  }
  else if (!status_str.compare(COMMAND_CONFIGURE_STRING))
  {
    status = LifeCycleCommand::CONFIGURE;
  }
  else if (!status_str.compare(COMMAND_CLEANUP_STRING))
  {
    status = LifeCycleCommand::CLEANUP;
  }
  else if (!status_str.compare(COMMAND_ACTIVATE_STRING))
  {
    status = LifeCycleCommand::ACTIVATE;
  }
  else if (!status_str.compare(COMMAND_DEACTIVATE_STRING))
  {
    status = LifeCycleCommand::DEACTIVATE;
  }
  else if (!status_str.compare(COMMAND_SHUTDOWN_STRING))
  {
    status = LifeCycleCommand::SHUTDOWN;
  }
  else if (!status_str.compare(COMMAND_DESTROY_STRING))
  {
    status = LifeCycleCommand::DESTROY;
  }
  else
  {
    return false;
  }
  return true;
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

const std::vector<LifeCycleState> AMLifeCycle::allLifeCycleStates()
{
  std::vector<LifeCycleState> all;
  for (int enumIndex = (int)LifeCycleState::INVALID; enumIndex <= (int)LifeCycleState::LAST_STATE; enumIndex++)
  {
    LifeCycleState state = static_cast<LifeCycleState>(enumIndex);
    all.push_back(state);
  }
  return all;
}

LifeCycleStatus AMLifeCycle::getStatus() const
{
  return status_;
}

void AMLifeCycle::setStatus(const LifeCycleStatus status)
{
  if (status_ < LifeCycleStatus::LAST_STATUS)
  {
    status_ = status;
  }
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
