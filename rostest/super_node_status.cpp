#include "ros/ros.h"
#include "std_msgs/String.h"
#include <gtest/gtest.h>  // googletest header file

void callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

TEST(TestNode, testSuperStatus)
{
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/super/status", 1000, callback);
  ros::Rate loop_rate(1); //1 Hz

  int count = 3;

  while(count--) {
    loop_rate.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "subscriber");
  return RUN_ALL_TESTS();
}