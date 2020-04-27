#include <super_lib/am_life_cycle.h>
#include <brain_box_msgs/LifeCycleState.h>

namespace am
{

//static constexpr std::string AMLifeCycle::STATE_INVALID_STRING;
//static constexpr std::string AMLifeCycle::STATE_UNCONFIGURED_STRING;

AMLifeCycle::AMLifeCycle() :
        nh_("~")
{
    state_ = LifeCycleState::UNCONFIGURED;
    status_ = LifeCycleStatus::OK;
    state_pub_ = nh_.advertise<brain_box_msgs::LifeCycleState>("/node_state",
            100);

    updater_.setHardwareID("none");
    updater_.broadcast(0, "Initializing node");
    updater_.add("diagnostics", this, &AMLifeCycle::addStatistics);
    updater_.force_update();

    heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0),
            &AMLifeCycle::heartbeatCB, this);
}

AMLifeCycle::~AMLifeCycle()
{
}

void AMLifeCycle::configure()
{
}

void AMLifeCycle::cleanup()
{
}

void AMLifeCycle::activate()
{
}

void AMLifeCycle::deactivate()
{
}

void AMLifeCycle::shutdown()
{
}

void AMLifeCycle::destroy()
{
}

void AMLifeCycle::onConfigure()
{
}

void AMLifeCycle::onCleanUp()
{
}

void AMLifeCycle::onActivate()
{
}

void AMLifeCycle::onDeactivate()
{
}

void AMLifeCycle::onShutdown()
{
}

void AMLifeCycle::onError()
{
}

void AMLifeCycle::addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw)
{
    stats_list_.addStatistics(dsw);
    LifeCycleStatus status = stats_list_.process();
    setStatus(status);
    dsw.summary((uint8_t)status, "update");
}

void AMLifeCycle::heartbeatCB(const ros::TimerEvent &event)
{
    updater_.force_update();

    ROS_INFO_STREAM("stats: " << stats_list_.getStatsStrShort());

    stats_list_.reset();

    brain_box_msgs::LifeCycleState msg;
    msg.node_name = ros::this_node::getName();
    msg.process_id = 0;
    msg.state = (uint8_t) state_;
    msg.status = (uint8_t) status_;
    msg.subsystem = "";
    msg.value = "";
    state_pub_.publish(msg);
}

const std::string_view &AMLifeCycle::stateToString(LifeCycleState state)
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
        return STATE_ACTIVATING_STRING;
    case LifeCycleState::ERROR_PROCESSING:
        return STATE_ERROR_PROCESSING_STRING;
    case LifeCycleState::INVALID:
    default:
        return STATE_INVALID_STRING;
    }
}

const std::string_view &AMLifeCycle::statusToString(LifeCycleStatus state)
{
    switch (state)
    {
    case LifeCycleStatus::OK:
        return STATUS_OK_STRING;
    case LifeCycleStatus::WARN:
        return STATUS_WARN_STRING;
    case LifeCycleStatus::ERROR:
    default:
        return STATUS_ERROR_STRING;
    }
}

LifeCycleState AMLifeCycle::getState() const
{
    return state_;
}

void AMLifeCycle::setState(const LifeCycleState state)
{
    state_ = state;
}

LifeCycleStatus AMLifeCycle::getStatus() const
{
    return status_;
}

void AMLifeCycle::setStatus(const LifeCycleStatus status)
{
    status_ = status;
}

}
;
