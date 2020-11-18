#include "rostest_transition.h" //FIXME: currently relative path

RostestTransition::RostestTransition() : loop_rate(1)
{
  ready = ready_after_armed = armed = in_auto = manual = disarming =
  semi_auto = auto_after_semi_auto = abort = shutdown = false;

  /* TODO: currently hardcoded to send only to super, make it general? */
  operatorCommandPublisher = n.advertise<brain_box_msgs::OperatorCommand>(am_super_topics::OPERATOR_COMMAND, 100);
  missionStateSubscription = n.subscribe(am_super_topics::SUPER_STATE, 1000, &RostestTransition::missionStateCallback, this);
}

void RostestTransition::missionStateCallback(const brain_box_msgs::VxState& msg)
{ 
  switch(msg.state)
  {
    case brain_box_msgs::VxState::READY:
      ROS_INFO_STREAM("READY received");
      if(armed)
      {
        ready_after_armed = true;
      }
      else
      {
        ready = true;
      }
      break;  
    case brain_box_msgs::VxState::SHUTDOWN:
      ROS_INFO_STREAM("SHUTDOWN received");
      shutdown = true;
      break;  
    case brain_box_msgs::VxState::ARMED:
      ROS_INFO_STREAM("ARMED received");
      armed = true;
      break;
    case brain_box_msgs::VxState::AUTO:
      ROS_INFO_STREAM("AUTO received");
      if(semi_auto)
        auto_after_semi_auto = true;
      else
        in_auto = true;
      break;
    case brain_box_msgs::VxState::DISARMING:
      ROS_INFO_STREAM("DISARMING received");
      disarming = true;
      break;
    case brain_box_msgs::VxState::MANUAL:
      ROS_INFO_STREAM("MANUAL received");
      manual = true;
      break;
    case brain_box_msgs::VxState::SEMI_AUTO:
      ROS_INFO_STREAM("SEMI-AUTO received");
      semi_auto = true;
      break;
    case brain_box_msgs::VxState::ABORT:
      ROS_INFO_STREAM("ABORT received");
      abort = true;
      break;
  }
}

void RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::_command_type cmd, bool& responded)
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
  ASSERT_TRUE(responded);
}

void RostestTransition::waitUntilResponseReceived(bool& responded)
{
  while(!responded && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ASSERT_TRUE(responded);
}