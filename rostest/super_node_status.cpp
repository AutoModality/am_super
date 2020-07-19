#include "ros/ros.h"
#include "std_msgs/String.h"
#include <gtest/gtest.h>  // googletest header file
#include <brain_box_msgs/VxState.h> //msg for status

int ARMED_count = 3;

void callback(const brain_box_msgs::VxState& msg)
{
  if(msg.state == brain_box_msgs::VxState::ARMED) 
  {
    ROS_INFO_STREAM("heartbeat received..");
    ARMED_count--;
  }
  else 
  {
    ROS_INFO_STREAM("heartbeat not received, retrying...");
  }
}


TEST(TestNode, testState) 
{
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/vstate/summary", 1000, callback);
  ros::Rate loop_rate(1); //1 Hz

  ROS_INFO_STREAM("Checking for heartbeat until 3 received (Ctrl-C to cancel)..\n");
  while(ARMED_count && ros::ok()) 
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  ROS_INFO_STREAM("System status: ARMED");
  ASSERT_EQ(0, ARMED_count);
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "subscriber");
  

  return RUN_ALL_TESTS();
}