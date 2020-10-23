#include "ros/ros.h"                          // ros header file
#include <gtest/gtest.h>                      // googletest header file
#include <brain_box_msgs/VxState.h>           // msg for status
#include <brain_box_msgs/OperatorCommand.h>   // to be armed, launch for state transitions
#include <super_lib/am_super_topics.h>        // reference topic names
#include <super_lib/am_life_cycle.h>

using namespace std;
using namespace am;
using namespace brain_box_msgs;

class AutoToManual : public ::testing::Test, am::AMLifeCycle 
{
public:

  bool armed, in_auto, manual;

  ros::NodeHandle n;
  ros::Subscriber missionStateSubscription;
  ros::Publisher operatorCommandPublisher;
  ros::Rate loop_rate;

  AutoToManual() : loop_rate(1)
  {
    armed = in_auto = manual = false;
    operatorCommandPublisher = n.advertise<brain_box_msgs::OperatorCommand>(am_super_topics::OPERATOR_COMMAND, 100);
    missionStateSubscription = n.subscribe(am_super_topics::SUPER_STATE, 1000, &AutoToManual::missionStateCallback, this);
  }

  /**
   * callback function for ROS test node whenever data is published
   *
   * @param msg custom message containing state information about am_super
   */
  void missionStateCallback(const brain_box_msgs::VxState& msg)
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

  void ASSERT_sendCommandUntilResponseReceived(OperatorCommand::_command_type cmd, bool& responded)
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
};

TEST_F(AutoToManual, testState_AutoToManual)
{
  ASSERT_sendCommandUntilResponseReceived(OperatorCommand::ARM, armed);
  ASSERT_sendCommandUntilResponseReceived(OperatorCommand::LAUNCH, in_auto);
  ASSERT_sendCommandUntilResponseReceived(OperatorCommand::MANUAL, manual);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "auto_to_manual");

  return RUN_ALL_TESTS();
}