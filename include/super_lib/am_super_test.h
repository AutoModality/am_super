#ifndef _AM_SUPER_SUPER_LIB_AM_SUPER_TEST_H_
#define _AM_SUPER_SUPER_LIB_AM_SUPER_TEST_H_

#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>             // googletest header file

#include <brain_box_msgs/msg/operator_command.hpp>  // to be armed, launch for state transitions
#include <brain_box_msgs/msg/controller_state.hpp>
#include <brain_box_msgs/msg/vx_state.hpp>  // msg for status
#include <brain_box_msgs/msg/life_cycle_state.hpp>  // msg for status
#include <super_lib/am_life_cycle.h>
#include <super_lib/am_life_cycle_types.h>
#include <super_lib/am_super_topics.h>
#include <super_lib/am_life_cycle.h>
#include <vb_util_lib/topics.h>

using namespace std;
using namespace am;

/**
 * Base class for testing the LifeCycle transitions of a target node along with the
 * state transition of the supervisor node.
 */
class AMSuperTest : public ::testing::Test
{
private:
  rclcpp::Subscription<brain_box_msgs::msg::LifeCycleState>::SharedPtr nodeLifeCycleStateSubscription_;
  rclcpp::Subscription<brain_box_msgs::msg::VxState>::SharedPtr missionStateSubscription_;
  rclcpp::Publisher<brain_box_msgs::msg::OperatorCommand>::SharedPtr operatorCommandPublisher_;
  rclcpp::Publisher<brain_box_msgs::msg::ControllerState>::SharedPtr controllerStatePublisher_;
  std::multimap<string,brain_box_msgs::msg::LifeCycleState> node_states_;
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
  rclcpp::Node::SharedPtr nh_;
  string target_node_name_;

  AMSuperTest();
  AMSuperTest(string target_node_name);

  /** when configured and Inactive, arm is sent to prepare all nodes for takeoff*/
  void arm();

  /** when armed, signals for the props to spin and takeoff */
  void launch();

  /** to transition super into disarming, flight must be completed by landing or canceled */
  void landed();

  /** to transition super into disarming from armed */
  void cancel();
   
  /** to transition super into semi-auto from auto */
  void pause();

  /** to transition super into auto from semi-auto */
  void resume();

  /** to transition super into abort while in flight mode */
  void abort();

  /** to transition super into manual from an auto mode*/
  void manual();

  /** to shut super down. Must be in BOOTING or READY */
  void shutdown();


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
  void nodeLifeCycleStateCallback(const brain_box_msgs::msg::LifeCycleState& msg);

  void missionStateCallback(const brain_box_msgs::msg::VxState& msg);

  /** Loop until a am_super is broadcasting the desired state or until
   * ros says its time to quit.
   * FIXME: export SuperState into a library and use instead of the untyped messages.
   * @param log_code to corrleate the log output to the source code 
   */
  void waitUntilMissionState(const uint8_t mssion_state, std::string log_code, float sleep = 1);
  
  [[deprecated("use waitUntilMissionState(status,log_code,sleep)")]]
  void waitUntilMissionState(const uint8_t mssion_state, float sleep = 1);

  /**
   * spin until the desired state is found or until the test times out.
   */
  void waitUntil(const LifeCycleState state, float sleep = 1);

  /**
   * @param log_code makes it easy to find the source of the log messages 
   */
  void waitUntil(const LifeCycleState state, const std::string log_code, float sleep = 1);


  /**
   * Wait until status is received or the test times out.
   */
  [[deprecated("use waitUntil(status,log_code,sleep)")]]
  void waitUntilStatus(const LifeCycleStatus& status, float sleep = 1);

  /**
   * @brief look for status periodically, based on the sleep, or timeout based on the test time limit.
   * @param log_code makes it easy to find the source of the log messages 
   */ 
  void waitUntil(const LifeCycleStatus& status, const std::string log_code,float sleep = 1);

  /**
   * Looks to see if the given status has ever been received. 
   */
  bool nodeStatusReceived(string node_name, LifeCycleStatus status);

 /**
   * Generate publishers and subscriptions. 
   */
  void createPubsSubs();

#define TEST_LOG( args, code) \
  do \
  { \
    RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1.0, args << " [" << code << "]"); \
  } while (0)
};

#endif