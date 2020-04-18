#include <super/AMLifeCycle.h>

namespace am
{


AMLifeCycle::AMLifeCycle()  : nh_("~")
{
	state_ = LifeCycleState::UNCONFIGURED;
	state_pub_ = nh_.advertise<brain_box_msgs::LifeCycleState>("/process/status",100);
	state_update_timer_ = nh_.createTimer(ros::Duration(1.0), &AMLifeCycle::stateUpdateCB, this);
}

AMLifeCycle::~AMLifeCycle()
{
}

/**
* @brief Returns the current state
* @return State that the node is in
*/
LifeCycleState AMLifeCycle::getLifeCycleState()
{
	return state_;
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

void AMLifeCycle::stateUpdateCB(const ros::TimerEvent& event)
{
	brain_box_msgs::LifeCycleState msg;
	msg.node_name = ros::this_node::getName();
	msg.state = brain_box_msgs::LifeCycleState::ACTIVE;
	msg.status = "";
	msg.value = "";
	msg.process_id = 0;
	state_pub_.publish(msg);
}


};
