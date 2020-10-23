#include "ros/ros.h"                          // ros header file
#include <gtest/gtest.h>                      // googletest header file
#include <brain_box_msgs/VxState.h>           // msg for status
#include <brain_box_msgs/OperatorCommand.h>   // to be armed, launch for state transitions
#include <super_lib/am_super_topics.h>        // reference topic names
#include <super_lib/am_life_cycle.h>
#include <am_super/transition_utility.h>

using namespace std;
using namespace am;
using namespace brain_box_msgs;

class AutoToManual : public ::testing::Test, am::AMLifeCycle 
{
public:
  TransitionUtility t;
};

TEST_F(AutoToManual, testState_AutoToManual)
{
  t.ASSERT_sendCommandUntilResponseReceived(OperatorCommand::ARM, t.armed);
  t.ASSERT_sendCommandUntilResponseReceived(OperatorCommand::LAUNCH, t.in_auto);
  t.ASSERT_sendCommandUntilResponseReceived(OperatorCommand::MANUAL, t.manual);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "auto_to_manual");

  return RUN_ALL_TESTS();
}