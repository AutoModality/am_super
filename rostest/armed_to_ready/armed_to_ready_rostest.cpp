#include "../transition_utility.h"

using namespace brain_box_msgs;

class ArmedToReady : public ::testing::Test, am::AMLifeCycle 
{
public:
  TransitionUtility t;
};

TEST_F(ArmedToReady, testState_ArmedToReady)
{
  t.sendCommandUntilResponseReceived(OperatorCommand::ARM, t.armed);
  ASSERT_TRUE(t.armed);

  t.sendCommandUntilResponseReceived(OperatorCommand::CANCEL, t.ready_after_armed);
  ASSERT_TRUE(t.ready_after_armed);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "armed_to_ready_rostest");

  return RUN_ALL_TESTS();
}