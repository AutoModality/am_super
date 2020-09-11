/**
 * ROS Test node to ensure a node can successfully communicate with am_super
 * and retrieve it's status. Specifically, this test checks if am_super is in
 * the ARMED state 3 times.
 * 
 * The test node inherits from LifeCycle and must properly go through the lifecycle
 * states which are necessary to be "ARMED".  Assertions will be made to ensure this node
 * received a command to visit each state. 
 *
 * Times out after rostest configured in launch file.
 *
 * Test fails if number of times we receive 'ARMED' from super is less than 3.
 */
#include "ros/ros.h"                 // ros header file
#include <gtest/gtest.h>             // googletest header file
#include <brain_box_msgs/VxState.h>  // msg for status
#include <brain_box_msgs/OperatorCommand.h>  // to be armed, launch for state transitions
#include <super_lib/am_life_cycle.h>
#include <super_lib/am_life_cycle_mediator.h>

using namespace std;
using namespace am;

/* SuperState - indicates if we received the command from super yet*/
bool booting = false;
bool ready = false; 
bool arming = false;
bool armed = false;
bool in_auto = false;
constexpr int CHECK_TIME = 3;
/* LifeCycle - indicates if we received the command yet for a nodde*/
bool super_unconfigured = false;
bool super_configuring = false;
bool super_inactive = false; 
bool super_active = false;
bool super_activating = false;

bool rostest_unconfigured = false;
bool rostest_configuring = false;
bool rostest_inactive = false;
bool rostest_active = false;
bool rostest_activating = false;

constexpr string_view THIS_NODE_NAME = "/life_cycle_rostest";

constexpr string_view CORRECT = "CORRECT";  // represents the correct result in test
string_view order_status = CORRECT;         // used in test to verify order_status is correct

AMLifeCycleMediator life_cycle_mediator_;

class LifeCycleNodeTest : public ::testing::Test, am::AMLifeCycle
{
  public:
    LifeCycleNodeTest()
    {
      ROS_INFO_STREAM("Constructing Lifecycle Node Test");
      //see launch file for "init_state" = UNCONFIGURED
    }

    void onConfigure()
    {
      AMLifeCycle::onConfigure();
      configured=true;
    }
    void onActivate()
    {
      if(!configured)
      {
        order_status = "ERROR: Attempting to activate before being configured";
      }
      AMLifeCycle::onActivate();
      activated=true;
    }

    void onCleanup()
    {
      AMLifeCycle::onCleanup();
      cleanedUp=true;
    }

    void onDeactivate()
    {
      AMLifeCycle::onDeactivate();
      deactivated=true;
    }

    void onDestroy()
    {
      AMLifeCycle::onDestroy();
      destroyed=true;
    }

    void onError()
    {
      AMLifeCycle::onError();
      errored=true;
    }

    void onShutdown()
    {
      AMLifeCycle::onShutdown();
      shutdown=true;
    }
    

  protected:
    bool configured = false;
    bool activated = false;
    bool cleanedUp = false;
    bool deactivated = false;
    bool destroyed = false;
    bool errored = false;
    bool shutdown = false;
};


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
  }
}

void nodeLifeCycleStateCallback(const brain_box_msgs::LifeCycleState& msg)
{ 
  LifeCycleState state = (LifeCycleState)msg.state;
  string_view state_string = life_cycle_mediator_.stateToString(state);
  ROS_INFO_STREAM("Node lifecycle state " << state_string << " received from " << msg.node_name);
  //FIXME: super's node name should come from a constant since it is also used in super
  if(msg.node_name.find("am_super") != std::string::npos)
  {
    switch(state)
    {
      case LifeCycleState::UNCONFIGURED:
        super_unconfigured = true;
        break;
      case LifeCycleState::CONFIGURING:
        super_configuring = true;
        break;
      case LifeCycleState::INACTIVE:
        super_inactive = true;
        break;
      case LifeCycleState::ACTIVE:
        super_active = true;
        break;
      case LifeCycleState::ACTIVATING:
        super_activating = true;
        break;
      default:
        ROS_WARN_STREAM(state_string << " unhandled for " << msg.node_name);
    }
  }
  else if(msg.node_name.find(THIS_NODE_NAME) != std::string::npos)
  {
     switch(state)
    {
      case LifeCycleState::UNCONFIGURED:
        rostest_unconfigured = true;
        break;
      case LifeCycleState::CONFIGURING:
        rostest_configuring = true;
        break;
      case LifeCycleState::INACTIVE:
        rostest_inactive = true;
        break;
      case LifeCycleState::ACTIVE:
        rostest_active = true;
        break;
      case LifeCycleState::ACTIVATING:
        rostest_activating = true;
        break;
      default:
        ROS_WARN_STREAM(state_string << " unhandled for " << msg.node_name);
    }
  }
  else
  {
    ROS_ERROR_STREAM(msg.node_name << " not recognized.");
  }
  
}

TEST_F(LifeCycleNodeTest, testState_SuperRemainsInREADY)
{
  ros::NodeHandle n;

  ros::Subscriber missionStateSubscription = n.subscribe("/vstate/summary", 1000, missionStateCallback);
  ros::Subscriber nodeLifeCycleStateSubscription = n.subscribe("/node_state", 100, nodeLifeCycleStateCallback);
  //FIXME: reference constant for "/operator/command"
  ros::Publisher operatorCommandPublisher = n.advertise<brain_box_msgs::OperatorCommand>("/operator/command",100);
  ros::Rate loop_rate(1);  // 1 Hz

  // check that we are booting
  {


    ROS_INFO_STREAM("Waiting to receive BOOTING from AMSuper (Ctrl-C to cancel)..\n");
    while (!booting && ros::ok() )
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

    ASSERT_TRUE(booting) << "/am_super SuperState did not report a BOOTING state";
    while (!super_unconfigured && ros::ok() )
    {
      ros::spinOnce();
      loop_rate.sleep();
    }    
    
    ASSERT_TRUE(super_unconfigured) << "/am_super LifeCycle did not report an UNCONFIGURED state"; 
    // while (!rostest_unconfigured && ros::ok() )
    // {
    //   ros::spinOnce();
    //   loop_rate.sleep();
    // }
    // ASSERT_TRUE(rostest_unconfigured) << "/life_cycle_rostest LifeCycle did not report an UNCONFIGURED state";
  }

  ROS_INFO_STREAM("BOOTING received ");

  //super must become active first
  {
    while (!super_inactive && ros::ok())
    {
      ROS_INFO_STREAM("Waiting for super to become inactive...");
      ros::spinOnce();
      loop_rate.sleep();
    }
    ROS_INFO_STREAM("Super is inactive ");
    ASSERT_TRUE(super_inactive) << "/am_super LifeCycle did not report an INACTIVE state"; 

    
    //not a great test since super may already be active
    //want to ensure that we aren't ready until super is active
    if(!super_active)
    {
      ASSERT_FALSE(ready);
    }
    
    while ((!super_active) && ros::ok())
    {
      ROS_INFO_STREAM("Waiting for super to become active...");
      ros::spinOnce();
      loop_rate.sleep();
    }
    ASSERT_TRUE(super_active) << "Super should activate itself after configuring.";
    ASSERT_TRUE(super_activating) << " Super is active, but never was activating";
  }

  
  // check that we are ready
  {
    ROS_INFO_STREAM("Waiting to receive READY from AMSuper (Ctrl-C to cancel)..\n");

    ASSERT_FALSE(arming) << "Super should not be arming yet";
    while ((!ready  || !rostest_inactive) && ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

    ASSERT_TRUE(ready) << "/am_super SuperState did not report a READY state";
    ASSERT_TRUE(rostest_inactive) << "/life_cycle_rostest LifeCycle did not report an INACTIVE state";
    ASSERT_FALSE(rostest_active) << "/life_cycle_rostest should not be active until arming";
  }

  // check that we are not arming yet, until operator command is sent
  {
    ASSERT_FALSE(arming) << "/am_super SuperState must wait for a trigger to transition to ARMING";
    ASSERT_FALSE(armed) << "/am_super SuperState should not be armed yet";
    ASSERT_EQ(order_status, CORRECT) << order_status;
    ASSERT_TRUE(configured) << "This node should be configured";
    ASSERT_FALSE(activated) << "This node should not be activated yet";
    ASSERT_FALSE(cleanedUp) << "This node should not have cleaned up yet";
    ASSERT_FALSE(deactivated) << "This node should not have deactivated yet";
    ASSERT_FALSE(destroyed) << "This node should not be destroyed yet";
    ASSERT_FALSE(errored) << "This node should not have an error right now";
    ASSERT_FALSE(shutdown) << "This node should not have shut down";
  }

  // now, let's arm it with the operator command
  {
    string_view node_name = THIS_NODE_NAME;
    brain_box_msgs::OperatorCommand armCommand;
    armCommand.node_name = node_name;
    armCommand.command = brain_box_msgs::OperatorCommand::ARM;
    operatorCommandPublisher.publish(armCommand);
    int cnt = 0;
    while(!arming && cnt < CHECK_TIME && ros::ok())
    {
      ros::spinOnce();
      cnt++;
      loop_rate.sleep();
    }    
    ASSERT_TRUE(arming) << "Super should now be arming";
  }

  //now it must go armed once all the nodes go active
  {
    int cnt = 0;
    while(!armed && cnt < CHECK_TIME && ros::ok())
    {
      ros::spinOnce();
      cnt++;
      loop_rate.sleep();
    }    
    ASSERT_TRUE(armed) << "/am_super SuperState should now be armed";
    ASSERT_TRUE(super_active) << "/am_super LifeCycle should still be active";
    ASSERT_TRUE(rostest_active) << "/life_cycle_rostest LifeCycle should now be active";
    ASSERT_TRUE(rostest_activating) << " Rostest is active, but never was activating";
    ASSERT_TRUE(activated) << "This node should now be activated";
    ASSERT_FALSE(cleanedUp) << "This node should not have cleaned up yet";
    ASSERT_FALSE(deactivated) << "This node should not have deactivated yet";
    ASSERT_FALSE(destroyed) << "This node should not be destroyed yet";
    ASSERT_FALSE(errored) << "This node should not have an error right now";
    ASSERT_FALSE(shutdown) << "This node should not have shut down";
  }

  // now let's launch it with the operator command
  {
    string_view node_name = THIS_NODE_NAME;
    brain_box_msgs::OperatorCommand armCommand;
    armCommand.node_name = node_name;
    armCommand.command = brain_box_msgs::OperatorCommand::LAUNCH;
    operatorCommandPublisher.publish(armCommand);
    
    int cnt = 0;
    while(!in_auto && cnt < CHECK_TIME && ros::ok())
    {
      ros::spinOnce();
      cnt++;
      loop_rate.sleep();
    }    
    ASSERT_TRUE(in_auto) << "/am_super SuperState should now be in AUTO";
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "subscriber");

  return RUN_ALL_TESTS();
}