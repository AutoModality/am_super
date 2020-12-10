/**
 * ROS Test for the happy case, emulating the acsl_flight_api by publishing mavros and acsl messages necessary to 
 * transition through a standard lifecycle.
 * 
 * On configure, this publishes mavros and health messages as if it were the flight api. This should make the flight monitor happy 
 * making the system ready.  
 * 
 * The test then sends the operator arm command so the monitor will start logging in the bag files and go active.
 * 
 * When ARMED, the launch command is sent and this sends the mavros message indicating ACSL OFFBOARD command is cruising
 * 
 * When Controller:Completed is sent, the monitor will close the bag files and go back to inactive state.
 * 
 */
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

/**The test node impersonating ACSL Flight API */
class TestFixture : public ::testing::Test, am::AMLifeCycle
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
    void publishOperatorCommand(uint8_t command)
    {
      brain_box_msgs::OperatorCommand msg;
      msg.node_name = ros::this_node::getName();
      msg.command = command;
      operatorCommandPublisher_.publish(msg);
    }

    /** Acting as the controller, this publishes the controller state 
     * to signal transitions due to autonomous processing
     * @param state one of the brain_box_msgs::ControllerState state enums
     * @see landed()
     * */
    void publishControllerState(uint8_t state)
    {
      brain_box_msgs::ControllerState msg;
      msg.node_name = ros::this_node::getName();
      msg.state = state;
      controllerStatePublisher_.publish(msg);
    }

  public:
    ros::NodeHandle n;
    string target_node_name_;
;
    TestFixture()
    {
      mavrosStatePublisher_ = n.advertise<mavros_msgs::State>(am::am_topics::MAVROS_STATE, 100);
      nodeLifeCycleStateSubscription_ = n.subscribe(am_super_topics::LIFECYCLE_STATE, 1000, &TestFixture::nodeLifeCycleStateCallback,this);
      missionStateSubscription_ = n.subscribe(am_super_topics::SUPER_STATE, 1000, &TestFixture::missionStateCallback,this);
      operatorCommandPublisher_ = n.advertise<brain_box_msgs::OperatorCommand>(am_super_topics::OPERATOR_COMMAND,100);
      controllerStatePublisher_ = n.advertise<brain_box_msgs::ControllerState>(am_super_topics::CONTROLLER_STATE, 100);

      target_node_name_= ros::this_node::getName();
    }

    /**Publish happy state on demand to act as the acs_flight_api */
    void publishState()
    {
      mavros_msgs::State state;
      state.connected = true;
      mavrosStatePublisher_.publish(state);
    }

    /** when configured and Inactive, arm is sent to prepare all nodes for takeoff*/
    void arm()
    {
      ROS_INFO("operator sending arm command");
      publishOperatorCommand(brain_box_msgs::OperatorCommand::ARM);
    }

    /** when armed, signals for the props to spin and takeoff */
    void launch()
    {
      ROS_INFO("operator sending launch command");
      publishOperatorCommand(brain_box_msgs::OperatorCommand::LAUNCH);
    }

    void landed()
    {
      ROS_INFO("controller sending landed state");
      publishControllerState(brain_box_msgs::ControllerState::COMPLETED);
    }

    /** searches the node states matching the lifecycle given. 
     * 
     * @return true if the state is found for the node given, false otherwise
     */
    bool nodeStateReceived(string node_name,LifeCycleState state)
    {
       if(node_states_.count(node_name)){
          int key = 2;     
          auto lower_it = node_states_.lower_bound(node_name); 
          auto upper_it = node_states_.upper_bound(node_name); 
            
          while (lower_it != upper_it)     
          { 
              if (lower_it -> first == node_name) {
                brain_box_msgs::LifeCycleState state_msg = lower_it -> second;
                if((LifeCycleState)state_msg.state == state){
                  return true;
                }
              }
              lower_it++; 
          }     
          return false; 
       }
    }

    /**
     * @return true if the desired state is anywhere in the list, regardless of order
     */
    bool missionStateReceived(uint8_t mission_state)
    {
      ROS_INFO("Mission Station: Received: %i  Total: %i",mission_state, (int) mission_states_.size());
      return mission_states_.back() == mission_state;
    }

    /**
     * Callback sniffing the state of nodes, as if it were am_super, to see 
     * if the target node is transitioning as expected. 
     * 
     * Simply registers states in multimap for later inspection.
     */
    void nodeLifeCycleStateCallback(const brain_box_msgs::LifeCycleState& msg)
    { 
      node_states_.emplace(msg.node_name,msg);
    }

    void missionStateCallback(const brain_box_msgs::VxState& msg)
    {
      mission_states_.insert(mission_states_.end(),msg.state);
    }

    /**Loop until a am_super is broadcasting the desired state or until
     * ros says its time to quit.
     * FIXME: export SuperState into a library and use instead of the untyped messages.
     */
    void waitUntilMissionState(const uint8_t mssion_state, float sleep=1)
    {
      ros::Rate loop_rate(sleep);
      while (!missionStateReceived(mssion_state) && ros::ok() )
      {
        ros::spinOnce();
        loop_rate.sleep();
      }
    }

    /**
     * spin until the desired state is found or until the test times out.
     */
    void waitUntil(const LifeCycleState state, float sleep=1){
      ros::Rate loop_rate(sleep);
      while (!nodeStateReceived(target_node_name_,state) && ros::ok() )
      {
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
};




TEST_F(TestFixture, primary_scenario)
{
  //wait for acsl super to come online and be ready to configure
  waitUntil(LifeCycleState::CONFIGURING);
  //acsl super is configuring and listening for mavros state message
  publishState();

  //acsl super should configure and transition to ready
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  arm();
  waitUntil(LifeCycleState::ACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::ARMED);
  launch();
  waitUntilMissionState(brain_box_msgs::VxState::AUTO);
  landed();
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  //the test launch file timeout acts as an assertion if any wait is blocked and unfinishing
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}