#ifndef _ROSTEST_TRANSITION_H_
#define _ROSTEST_TRANSITION_H_

#include "ros/ros.h"        
#include <gtest/gtest.h>                  // ros header file       
#include <brain_box_msgs/VxState.h>           // msg for status
#include <brain_box_msgs/OperatorCommand.h>   // to be armed, launch for state transitions
#include <super_lib/am_super_topics.h>        // reference topic names
#include <super_lib/am_life_cycle.h>

using namespace am;
using namespace brain_box_msgs;

/**
 * This class encapsulates the data and methods needed to test supervisor transitioning through states.
 * Therefore, we can use this between different rostests and different transitions
 */
class RostestTransition : public ::testing::Test, am::AMLifeCycle 
{
public:
  //Indicates if we received the respective SuperState from super
  bool ready, armed, in_auto, manual, ready_after_armed, disarming,
  semi_auto;

  //node handle that allows us to publish and subscribe
  ros::NodeHandle n;

  //to receive SuperState from super
  ros::Subscriber missionStateSubscription;

  //to send operatorCommand to super
  ros::Publisher operatorCommandPublisher;

  //the rate that we send messages at
  ros::Rate loop_rate;

  RostestTransition();
  /**
   * callback function for ROS test node whenever data is published
   *
   * @param msg custom message containing state ijjnformation about am_super
   */
  void missionStateCallback(const brain_box_msgs::VxState& msg);

  /**
   * Sends OperatorCommand to super until 'responded' is true (set by missionStateCallback)
   * 
   * @param cmd Operator Command that we will be publishing to operatorCommandPublisher
   * @param responded One of the bool member variables associated with this class.
   * 
   */
  void sendCommandUntilResponseReceived(OperatorCommand::_command_type cmd, bool& responded);

    /**
   * Sends OperatorCommand to super until 'responded' is true (set by missionStateCallback)
   * 
   * @param responded One of the bool member variables associated with this class. When this become true, we 
   * received the response
   * 
   */
  void waitUntilResponseReceived(bool& responded);
};

#endif