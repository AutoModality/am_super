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
#include <super_lib/am_life_cycle.h>

using namespace std;
using namespace am;

bool ready = false; //indicates if we received READY from super
bool arming = false;
constexpr int CHECK_TIME = 10;

constexpr string_view CORRECT = "CORRECT";  // represents the correct result in test
string_view order_status = CORRECT;         // used in test to verify order_status is correct

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
void callback(const brain_box_msgs::VxState& msg)
{ 
  if (msg.state == brain_box_msgs::VxState::READY)
  {
    ROS_INFO_STREAM("Ready received.");
    ready = true;
  }
  else if(msg.state == brain_box_msgs::VxState::ARMING)
  {
    ROS_INFO_STREAM("Arming received");
    arming = true;
  }
  else
  {
    ROS_INFO_STREAM("Neither Ready nor ARMING received, retrying..");
  }
}

TEST_F(LifeCycleNodeTest, testState_SuperRemainsInREADY)
{
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/vstate/summary", 1000, callback);
  ros::Rate loop_rate(1);  // 1 Hz

  ROS_INFO_STREAM("Waiting to receive READY from AMSuper (Ctrl-C to cancel)..\n");
  while (!ready && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  ASSERT_TRUE(ready) << "Super did not report a READY state or ros::shutdown() was called";

  ROS_INFO_STREAM("Ensure super remains in READY for atleast 10 seconds:");

  int cnt = 0;
  while(!arming && cnt < CHECK_TIME && ros::ok())
  {
    ros::spinOnce();
    cnt++;
    loop_rate.sleep();
  }

  EXPECT_FALSE(arming) << "ERROR: Super must wait for a trigger to transition to ARMING";

  ASSERT_EQ(order_status, CORRECT) << order_status;
  EXPECT_TRUE(configured);
  EXPECT_FALSE(activated) << "ERROR: This node should not be activated yet";
  EXPECT_FALSE(cleanedUp);
  EXPECT_FALSE(deactivated);
  EXPECT_FALSE(destroyed);
  EXPECT_FALSE(errored);
  EXPECT_FALSE(shutdown);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "subscriber");

  return RUN_ALL_TESTS();
}