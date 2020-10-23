#include <am_super/transition_utility.h>

TransitionUtility::TransitionUtility() : loop_rate(1)
{
  armed = in_auto = manual = false;
  operatorCommandPublisher = n.advertise<brain_box_msgs::OperatorCommand>(am_super_topics::OPERATOR_COMMAND, 100);
  missionStateSubscription = n.subscribe(am_super_topics::SUPER_STATE, 1000, &TransitionUtility::missionStateCallback, this);
}

/**
 * callback function for ROS test node whenever data is published
 *
 * @param msg custom message containing state information about am_super
 */
void TransitionUtility::missionStateCallback(const brain_box_msgs::VxState& msg)
{ 
  switch(msg.state)
  {
    case brain_box_msgs::VxState::ARMED:
      ROS_INFO_STREAM("ARMED received");
      armed = true;
      break;
    case brain_box_msgs::VxState::AUTO:
      ROS_INFO_STREAM("AUTO received");
      in_auto = true;
      break;
    case brain_box_msgs::VxState::MANUAL:
      ROS_INFO_STREAM("MANUAL received");
      manual = true;
      break;
  }
}

void TransitionUtility::ASSERT_sendCommandUntilResponseReceived(OperatorCommand::_command_type cmd, bool& responded)
{
  OperatorCommand msg;
  msg.node_name = ros::this_node::getName();
  msg.command = cmd;

  while(!responded && ros::ok())
  {
    operatorCommandPublisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
