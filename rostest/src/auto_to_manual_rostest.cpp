#include "ros/ros.h"                          // ros header file
#include <gtest/gtest.h>                      // googletest header file
#include <brain_box_msgs/VxState.h>           // msg for status
#include <brain_box_msgs/OperatorCommand.h>   // to be armed, launch for state transitions
#include <super_lib/am_super_topics.h>        // reference topic names
#include <super_lib/am_life_cycle.h>

using namespace std;
using namespace am;

constexpr string_view THIS_NODE_NAME = "/auto_to_manual_rostest";

bool booting = false,
ready = false,
arming= false,
armed= false,
in_auto= false,
manual= false;

class TransitionTest : public ::testing::Test, am::AMLifeCycle {};

/**
 * callback function for ROS test node whenever data is published
 *
 * @param msg custom message containing state information about am_super
 */
void missionStateCallback(const brain_box_msgs::VxState& msg)
{ 
  switch(msg.state)
  {
    case brain_box_msgs::VxState::BOOTING:
      ROS_INFO_STREAM("BOOTING received");
      booting = true;
      break;
    case brain_box_msgs::VxState::READY:
      ROS_INFO_STREAM("READY received");
      ready = true;
      break;
    case brain_box_msgs::VxState::ARMING:
      ROS_INFO_STREAM("ARMING received");
      arming = true;
      break;
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

TEST_F(TransitionTest, testState_AutoToManual)
{
  ros::NodeHandle n;
  ros::Subscriber missionStateSubscription = n.subscribe(am_super_topics::SUPER_STATE, 1000, missionStateCallback);
  ros::Publisher operatorCommandPublisher = n.advertise<brain_box_msgs::OperatorCommand>(am_super_topics::OPERATOR_COMMAND, 100);
  ros::Rate loop_rate(1); //1 Hz

  brain_box_msgs::OperatorCommand command;
  command.node_name = THIS_NODE_NAME;

  //Super transitions into ready on its own, send arm command for READY->ARMING
  command.command = brain_box_msgs::OperatorCommand::ARM;

  while(!ready && ros::ok())
  {
    operatorCommandPublisher.publish(command);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "auto_to_manual");

  return RUN_ALL_TESTS();
}