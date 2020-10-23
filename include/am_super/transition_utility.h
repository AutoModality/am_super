#include "ros/ros.h"                          // ros header file       
#include <brain_box_msgs/VxState.h>           // msg for status
#include <brain_box_msgs/OperatorCommand.h>   // to be armed, launch for state transitions
#include <super_lib/am_super_topics.h>        // reference topic names
#include <super_lib/am_life_cycle.h>

using namespace std;
using namespace am;
using namespace brain_box_msgs;

/**
 * This class encapsulates the data and methods needed to test supervisor transitioning through states.
 * Therefore, we can use this between different rostests and different transitions
 */
class TransitionUtility
{
public:

  bool armed, in_auto, manual;

  ros::NodeHandle n;
  ros::Subscriber missionStateSubscription;
  ros::Publisher operatorCommandPublisher;
  ros::Rate loop_rate;

  TransitionUtility();
  /**
   * callback function for ROS test node whenever data is published
   *
   * @param msg custom message containing state information about am_super
   */
  void missionStateCallback(const brain_box_msgs::VxState& msg);

  /**
   * Sends OperatorCommand to super until 'responded' is true (set by missionStateCallback)
   * 
   * @param cmd Operator Command that we will be publishing to operatorCommandPublisher
   * @param responded One of the bool member variables associated with this class.
   * 
   */
  void ASSERT_sendCommandUntilResponseReceived(OperatorCommand::_command_type cmd, bool& responded);
};