/**
 * ROS Test node to ensure a node can successfully communicate with am_super
 * and retrieve it's status. Specifically, this test checks if am_super is in
 * the ARMED state 3 times.
 *
 * Times out after rostest default 60 seconds.
 *
 * Test fails if number of times we receive 'ARMED' from super is less than 3.
 */
#include "ros/ros.h"                 // ros header file
#include <gtest/gtest.h>             // googletest header file
#include <brain_box_msgs/VxState.h>  // msg for status
#include <super_lib/am_life_cycle.h>


const int TARGET_COUNT = 3;  // number of 'ARMED' responses needed to pass test
int armed_count = 0;         // current number of received 'ARMED'

class LifecycleNodeTest : public ::testing::Test, am::AMLifeCycle
{
public:
  LifecycleNodeTest()
  {
    ROS_INFO_STREAM("Constructing Lifecycle Node Test");
  }
  void onConfigure()
  {
    ROS_INFO_STREAM("Being told to configure");
    AMLifeCycle::onConfigure();
    ROS_INFO_STREAM("Asked parent to configure");
  }
protected:
};


/**
 * callback function for ROS test node whenever data is published
 *
 * @param msg custom message containing state information about am_super
 */
void callback(const brain_box_msgs::VxState& msg)
{
  if (msg.state == brain_box_msgs::VxState::ARMED)
  {
    ROS_INFO_STREAM("heartbeat received..");
    armed_count++;
  }
  else
  {
    ROS_INFO_STREAM("heartbeat not received, retrying...");
  }
}

TEST_F(LifecycleNodeTest, testState)
{
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/vstate/summary", 1000, callback);
  ros::Rate loop_rate(1);  // 1 Hz

  ROS_INFO_STREAM("Checking for heartbeat until 3 received (Ctrl-C to cancel)..\n");
  while (armed_count < TARGET_COUNT && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ASSERT_EQ(armed_count, TARGET_COUNT);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "subscriber");

  return RUN_ALL_TESTS();
}