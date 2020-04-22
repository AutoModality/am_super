#include <am_super/am_life_cycle.h>
#include <brain_box_msgs/LifeCycleState.h>

namespace am
{


AMLifeCycle::AMLifeCycle()  : nh_("~")
{
	state_ = LifeCycleState::UNCONFIGURED;
	status_ = LifeCycleStatus::OK;
	state_pub_ = nh_.advertise<brain_box_msgs::LifeCycleState>("/node_state",100);
	heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0), &AMLifeCycle::heartbeatCB, this);
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

void AMLifeCycle::heartbeatCB(const ros::TimerEvent& event)
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


};
