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
bool ready = false; 
bool arming = false;
bool booting = false;
bool armed = false;
constexpr int CHECK_TIME = 3;
/* LifeCycle - indicates if we received the command yet for a nodde*/
bool super_unconfigured = false;
bool super_inactive = false; 
bool super_active = false;

bool rostest_unconfigured = false;
bool rostest_inactive = false;
bool rostest_active = false;


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
  if(msg.state == brain_box_msgs::VxState::BOOTING)
  {
    ROS_INFO_STREAM("BOOTING received");
    booting = true;
  }
  else if (msg.state == brain_box_msgs::VxState::READY)
  {
    ROS_INFO_STREAM("READY received");
    ready = true;
  }
  else if(msg.state == brain_box_msgs::VxState::ARMING)
  {
    ROS_INFO_STREAM("ARMING received");
    arming = true;
  }  
  else if(msg.state == brain_box_msgs::VxState::ARMED)
  {
    ROS_INFO_STREAM("ARMED received");
    armed = true;
  }
}

void nodeLifeCycleStateCallback(const brain_box_msgs::LifeCycleState& msg)
{ 
  LifeCycleState state = (LifeCycleState)msg.state;
  string_view state_string = life_cycle_mediator_.stateToString(state);
  ROS_INFO_STREAM("Node lifecycle state " << state_string << " received from " << msg.node_name);
  if(msg.node_name == "/am_super")
  {
    switch(state)
    {
      case LifeCycleState::UNCONFIGURED:
        super_unconfigured = true;
        break;
      case LifeCycleState::INACTIVE:
        super_inactive = true;
        break;
      case LifeCycleState::ACTIVE:
        super_active = true;
        break;
      default:
            ROS_WARN_STREAM("State not handled");      
    }
  }
  else
  {
     switch(state)
    {
      case LifeCycleState::UNCONFIGURED:
        rostest_unconfigured = true;
        break;
      case LifeCycleState::INACTIVE:
        rostest_inactive = true;
        break;
      case LifeCycleState::ACTIVE:
        rostest_active = true;
        break;
      default:
            ROS_WARN_STREAM("State not handled");

    }
  }
}

TEST_F(LifeCycleNodeTest, testState_SuperRemainsInREADY)
{
  ros::NodeHandle n;

  ros::Subscriber missionStateSubscription = n.subscribe("/vstate/summary", 1000, missionStateCallback);
  ros::Subscriber nodeLifeCycleStateSubscription = n.subscribe("/node_state", 1000, nodeLifeCycleStateCallback);
  //FIXME: reference constant for "/operator/command"
  ros::Publisher operatorCommandPublisher = n.advertise<brain_box_msgs::OperatorCommand>("/operator/command",100);
  ros::Rate loop_rate(1);  // 1 Hz

  ROS_INFO_STREAM("Waiting to receive BOOTING from AMSuper (Ctrl-C to cancel)..\n");
  while ((!booting || !super_unconfigured || !rostest_unconfigured) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  EXPECT_TRUE(booting) << "/am_super SuperState did not report a BOOTING state";
  EXPECT_TRUE(super_unconfigured) << "/am_super LifeCycle did not report an UNCONFIGURED state"; 
  EXPECT_TRUE(rostest_unconfigured) << "/life_cycle_rostest LifeCycle did not report an UNCONFIGURED state";

  ROS_INFO_STREAM("Waiting to receive READY from AMSuper (Ctrl-C to cancel)..\n");

  ready = super_inactive = rostest_inactive = false;

  EXPECT_FALSE(arming);
  while ((!ready || !super_inactive || !rostest_inactive) && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  EXPECT_TRUE(ready) << "/am_super SuperState did not report a READY state";
  EXPECT_TRUE(super_inactive) << "/am_super LifeCycle did not report an INACTIVE state"; 
  EXPECT_TRUE(rostest_inactive) << "/life_cycle_rostest LifeCycle did not report an INACTIVE state";
  EXPECT_FALSE(super_active);
  EXPECT_FALSE(rostest_active);
  ROS_INFO_STREAM("Ensure super remains in READY for atleast " << CHECK_TIME << " seconds:");

  {
    EXPECT_FALSE(arming);
    EXPECT_FALSE(armed);
    int cnt = 0;
    while(!arming && cnt < CHECK_TIME && ros::ok())
    {
      ros::spinOnce();
      cnt++;
      loop_rate.sleep();
    }
  }

  EXPECT_FALSE(arming) << "ERROR: Super must wait for a trigger to transition to ARMING";
  EXPECT_FALSE(armed);
  EXPECT_EQ(order_status, CORRECT) << order_status;
  EXPECT_TRUE(configured);
  EXPECT_FALSE(activated) << "ERROR: This node should not be activated yet";
  EXPECT_FALSE(cleanedUp);
  EXPECT_FALSE(deactivated);
  EXPECT_FALSE(destroyed);
  EXPECT_FALSE(errored);
  EXPECT_FALSE(shutdown);

  // now, let's arm it
  {
    string node_name="/life_cycle_node_test";
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
    EXPECT_TRUE(arming);
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
    EXPECT_TRUE(armed);
    EXPECT_TRUE(activated) << "ERROR: This node should now be";
    EXPECT_TRUE(super_active);
    EXPECT_TRUE(rostest_active);
  }




}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "subscriber");

  return RUN_ALL_TESTS();
}