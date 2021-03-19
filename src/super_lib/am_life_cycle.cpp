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
  //FIXME: This string should come from the enum
  // always prefix with life_cycle in the parent to avoid namespace collisions with child and clarity in definition
  // param would be: `/am_child_node/life_cycle/some_param` so no conflict with `/am_child_node/some_param`
  std::string param_prefix="life_cycle/";
  
  //deprecated - prefix with life_cycle instead
  std::string default_state = "UNCONFIGURED";
  param<std::string>("init_state", init_state_str, default_state);
  param<std::string>(param_prefix + "init_state", init_state_str, default_state);
  
  //deprecated - prefix with life_cycle instead
  param<int>("configure_tolerance_s", configure_tolerance_s, 10);
  param<int>(param_prefix + "configure_tolerance_s", configure_tolerance_s, configure_tolerance_s);

  LifeCycleState init_state;
  if (life_cycle_mediator_.stringToState(init_state_str, init_state))
  {
    life_cycle_info_.state = init_state;
  }
  else
  {
    life_cycle_info_.state = LifeCycleState::ACTIVE;
  }
  life_cycle_info_.status = LifeCycleStatus::OK;
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

template<typename T>
bool AMLifeCycle::param(const std::string& param_name, T& param_val, const T& default_val) const
{
    bool result = nh_.param<T>(param_name, param_val, default_val);
    ROS_INFO_STREAM(param_name << " = " << param_val);
    return result;
}

void AMLifeCycle::lifecycleCB(const brain_box_msgs::LifeCycleCommand::ConstPtr msg)
{
  ROS_DEBUG_STREAM_THROTTLE(1.0, life_cycle_mediator_.commandToString((LifeCycleCommand)msg->command));

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
        configure();
        break;
      case LifeCycleCommand::CREATE:
        ROS_WARN_STREAM("illegal command " << life_cycle_mediator_.commandToString(LifeCycleCommand::CREATE));
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
  if (life_cycle_info_.state == initial_state)
  {
    ROS_INFO_STREAM(transition_name << ", current state: " << life_cycle_mediator_.stateToString(life_cycle_info_.state));
    setState(transition_state);
    on_function();
  }
  else if (life_cycle_info_.state == transition_state || life_cycle_info_.state == final_state)
  {
    ROS_DEBUG_STREAM("ignoring redundant " << transition_name);
  }
  else
  {
    ROS_WARN_STREAM("received illegal " << transition_name  << " in state " << life_cycle_mediator_.stateToString(life_cycle_info_.state));
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
  ROS_INFO("onConfigure called [POMH]");
  if(stats_list_.hasStats())
  {
    LifeCycleStatus status = stats_list_.process(throttle_info_.warn_throttle_s, throttle_info_.error_throttle_s);
    if(status != LifeCycleStatus::ERROR)
    {
      doConfigure(true);
    }
    else if (!withinConfigureTolerance())
    {
      ROS_WARN_STREAM_THROTTLE(5, stats_list_.getStatsStr() << " blocked by stats past configure tolerance with status " << life_cycle_mediator_.statusToString(status) );
    }
  }
  //if there are no stats and request to configure, then configure
  else
  {
    doConfigure(true);
  }
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
    ROS_INFO_STREAM("LifeCycle: " << life_cycle_mediator_.stateToString(life_cycle_info_.state));
}

void AMLifeCycle::doDeactivate(bool success)
{
  doTransition("deactivation", success, LifeCycleState::INACTIVE, LifeCycleState::ACTIVE);
}

void AMLifeCycle::configure()
{
  //mark the configuration start time once 
  if(getState() != LifeCycleState::CONFIGURING)
  {
    configure_start_time_=ros::Time().now();
  }
  transition("configure", LifeCycleState::UNCONFIGURED, LifeCycleState::CONFIGURING, LifeCycleState::INACTIVE,
  std::bind(&AMLifeCycle::onConfigure, this));
}

void AMLifeCycle::destroy()
{
  if (life_cycle_mediator_.illegalDestroy(life_cycle_info_))
  {
    ROS_INFO_STREAM("received illegal activate in state " << life_cycle_mediator_.stateToString(life_cycle_info_.state));
  }
  /* This condition is hit only if state equals FINALIZED. Checking SHUTTING_DOWN is redundant */
  else
  {
    ROS_INFO_STREAM("current state: " << life_cycle_mediator_.stateToString(life_cycle_info_.state));
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

bool AMLifeCycle::withinConfigureTolerance()
{
  bool tolerated = false;
  //outside of configuring, we have no tolerance
  if(life_cycle_info_.state == LifeCycleState::CONFIGURING)
  {
    ros::Duration duration_since_configure = ros::Time::now() - configure_start_time_;
    if (duration_since_configure <= ros::Duration(configure_tolerance_s) )
    {
      tolerated = true;
    }
  }
  return tolerated;
}
void AMLifeCycle::error(std::string error_code, bool forced)
{
  std::string error_code_message=" [" + error_code + "] ";
  if (!forced && life_cycle_mediator_.redundantError(life_cycle_info_))
  {
    ROS_WARN_STREAM("Error called again, but previously reported." << error_code_message);
  }
  else if(!forced && withinConfigureTolerance())
  {
    ROS_WARN_STREAM_THROTTLE(throttle_info_.warn_throttle_s,"Ignoring error" << error_code_message << "during configure tolerance of " << configure_tolerance_s << " seconds [GFRT]");
  }
  else
  {
    std::string forced_prefix = forced?"Forced ":"";
    ROS_ERROR_STREAM(forced_prefix << "Error" << error_code_message << "called while in: " << life_cycle_mediator_.stateToString(life_cycle_info_.state) << " [R45Y]" );
    setState(LifeCycleState::ERROR_PROCESSING);
    setStatus(LifeCycleStatus::ERROR);
    onError();
  }
}

void AMLifeCycle::onError()
{
  logState();
  doError(false);
}

void AMLifeCycle::doError(bool success)
{
  logState();
  if (success)
  {
    life_cycle_info_.state = LifeCycleState::UNCONFIGURED;
  }
  else
  {
    life_cycle_info_.state = LifeCycleState::FINALIZED;
  }
  sendNodeUpdate();
}

void AMLifeCycle::shutdown()
{
  if (life_cycle_mediator_.shutdown(life_cycle_info_))
  {
    ROS_INFO_STREAM("current state: " << life_cycle_mediator_.stateToString(life_cycle_info_.state));
    setState(LifeCycleState::SHUTTING_DOWN);
    onShutdown();
  }
  else if (life_cycle_mediator_.redundantShutdown(life_cycle_info_))
  {
    ROS_DEBUG_STREAM("ignoring redundant shutdown");
  }
  else
  {
    ROS_INFO_STREAM("received illegal activate in state " << life_cycle_mediator_.stateToString(life_cycle_info_.state));
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
  LifeCycleStatus status = stats_list_.process(throttle_info_.warn_throttle_s, throttle_info_.error_throttle_s);
  if(life_cycle_mediator_.statusError(status))
  {
    error("PQAE");
  }
  else
  {
    setStatus(status);

    //configuring is a special case where tolerance for errors is allowed
    if(getState() == LifeCycleState::CONFIGURING)
    {
      onConfigure();
    }
  }
  dsw.summary((uint8_t)status, "update");
}

AMStatReset& AMLifeCycle::configureHzStats(AMStatReset& stats, const int target_default)
{
    int hz_target;
    int hz_min_error;
    int hz_min_warn;
    int hz_max_warn;
    int hz_max_error;
    const std::string prefix = stats.getShortName();
    std::string hz_prefix=prefix + "/hz/";
    param<int>(hz_prefix + "target", hz_target, target_default);
    //give 5% tolerance in either direction for warning, 10% for error.  Override default values as desired
    const int warning_offset = std::ceil(hz_target * 0.05);
    const int error_offset =  2 * warning_offset;
    //don't go  below zero because that doesn't make any sense for hz.
    param<int>(hz_prefix + "error/min", hz_min_error, std::max(0,hz_target - error_offset));
    param<int>(hz_prefix + "warn/min",  hz_min_warn,  std::max(0,hz_target - warning_offset));
    param<int>(hz_prefix + "warn/max",  hz_max_warn,  hz_target + warning_offset);
    param<int>(hz_prefix + "error/max", hz_max_error, hz_target + error_offset);
    stats.setWarnError(hz_min_error, hz_min_warn, hz_max_warn, hz_max_error); 
    return stats;       
}
void AMLifeCycle::sendNodeUpdate()
{
  brain_box_msgs::LifeCycleState msg;
  msg.node_name = ros::this_node::getName();
  msg.process_id = 0;
  msg.state = (uint8_t)life_cycle_info_.state;
  msg.status = (uint8_t)life_cycle_info_.status;
  msg.subsystem = "";
  msg.value = "";
  state_pub_.publish(msg);
}


void AMLifeCycle::heartbeatCB(const ros::TimerEvent& event)
{
  updater_.force_update();

  std::stringstream ss;
  ss << life_cycle_mediator_.stateToString(life_cycle_info_.state) << "," << life_cycle_mediator_.statusToString(life_cycle_info_.status) << ","
     << stats_list_.getStatsStrShort();

  double throttle_s = getThrottle();
  ROS_INFO_STREAM_THROTTLE(throttle_s, "LifeCycle heartbeat: " << ss.str());

  stats_list_.reset();

  sendNodeUpdate();
}

LifeCycleState AMLifeCycle::getState() const
{
  return life_cycle_mediator_.getState(life_cycle_info_);
}

const std::string_view& AMLifeCycle::getStateName()
{
  return life_cycle_mediator_.stateToString(getState());
}

void AMLifeCycle::setState(const LifeCycleState state)
{
  LifeCycleState initial_state = life_cycle_info_.state;

  if (life_cycle_mediator_.setState(state, life_cycle_info_))
  {
    ROS_INFO_STREAM("changing state from " << life_cycle_mediator_.stateToString(initial_state) << " to " << life_cycle_mediator_.stateToString(state));
    sendNodeUpdate();
  }
  else
  {
    ROS_ERROR_STREAM("illegal state: " << (int)state);
  }
}

LifeCycleStatus AMLifeCycle::getStatus() const
{
  return life_cycle_mediator_.getStatus(life_cycle_info_);
}

bool AMLifeCycle::setStatus(const LifeCycleStatus status)
{
  //if we are in error and want to leave it
  if(life_cycle_info_.status == LifeCycleStatus::ERROR && status != LifeCycleStatus::ERROR)
  {
    ROS_WARN_STREAM_THROTTLE(getThrottle(), "requested to change status from ERROR to " << life_cycle_mediator_.statusToString(status));
  }

  else if (life_cycle_mediator_.setStatus(status, life_cycle_info_))
  {
    sendNodeUpdate();
  }

  else
  {
    ROS_ERROR_STREAM("illegal status: " << life_cycle_mediator_.statusToString(status));
  }
}

const std::string_view& AMLifeCycle::getStatusName()
{
  return life_cycle_mediator_.statusToString(getStatus());
}


//Is this being used?
void AMLifeCycle::setThrottleS(const double throttleS)
{
  return life_cycle_mediator_.setThrottleS(throttleS, throttle_info_);
}

double AMLifeCycle::getThrottle()
{
  return life_cycle_mediator_.getThrottle(life_cycle_info_, throttle_info_);
}


};

