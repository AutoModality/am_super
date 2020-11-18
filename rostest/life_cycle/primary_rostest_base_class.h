#ifndef _PRIMARY_ROSTEST_BASE_CLASS_H_
#define _PRIMARY_ROSTEST_BASE_CLASS_H_

#include "ros/ros.h"                 // ros header file
#include <gtest/gtest.h>             // googletest header file
#include <super_lib/am_life_cycle.h>
#include <super_lib/am_life_cycle_types.h>
#include <mavros_msgs/State.h>
#include <vb_util_lib/topics.h>
#include <super_lib/am_super_topics.h>
#include <super_lib/am_life_cycle.h>
#include <brain_box_msgs/OperatorCommand.h>  // to be armed, launch for state transitions
#include <brain_box_msgs/ControllerState.h>
#include <brain_box_msgs/VxState.h>  // msg for status

using namespace std;
using namespace am;

/**
 * Base class for testing the LifeCycle transitions of a target node along with the
 * state transition of the supervisor node.
 */
class PrimaryRostestBaseClass : public ::testing::Test, am::AMLifeCycle 
{
private:
  ros::Subscriber nodeLifeCycleStateSubscription_;
  ros::Subscriber missionStateSubscription_;
  ros::Publisher operatorCommandPublisher_;
  ros::Publisher controllerStatePublisher_;
  ros::Publisher mavrosStatePublisher_;
  std::multimap<string,brain_box_msgs::LifeCycleState> node_states_;
  std::vector<uint8_t> mission_states_;

  /** Act as the operator (typically via a remote or ground station) to send one
   * signal to transition to proceed, cancel, abort, etc.
   * 
   * @param command one of brain_box_msgs::OperatorCommand::ARM;
   * 
   * @see arm()
   * @see launch()
   */
  void publishOperatorCommand(uint8_t command);

  /** Acting as the controller, this publishes the controller state 
   * to signal transitions due to autonomous processing
   * @param state one of the brain_box_msgs::ControllerState state enums
   * @see landed()
   * */
  void publishControllerState(uint8_t state);

public:
  ros::NodeHandle n;
  string target_node_name_;

  PrimaryRostestBaseClass(string target_node_name);

  /**Publish happy state on demand to act as the acs_flight_api */
  void publishState();

  /** when configured and Inactive, arm is sent to prepare all nodes for takeoff*/
  void arm();

  /** when armed, signals for the props to spin and takeoff */
  void launch();

  void landed();

  /** searches the node states matching the lifecycle given. 
   * 
   * @return true if the state is found for the node given, false otherwise
   */
  bool nodeStateReceived(string node_name,LifeCycleState state);

  /**
   * @return true if the desired state is anywhere in the list, regardless of order
   */
  bool missionStateReceived(uint8_t mission_state);

  /**
   * Callback sniffing the state of nodes, as if it were am_super, to see 
   * if the target node is transitioning as expected. 
   * 
   * Simply registers states in multimap for later inspection.
   */
  void nodeLifeCycleStateCallback(const brain_box_msgs::LifeCycleState& msg);

  void missionStateCallback(const brain_box_msgs::VxState& msg);

  /**Loop until a am_super is broadcasting the desired state or until
   * ros says its time to quit.
   * FIXME: export SuperState into a library and use instead of the untyped messages.
   */
  void waitUntilMissionState(const uint8_t mssion_state, float sleep = 1);

  /**
   * spin until the desired state is found or until the test times out.
   */
  void waitUntil(const LifeCycleState state, float sleep = 1);
};

#endif