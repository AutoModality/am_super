#include "../transition_utility.h"

using namespace brain_box_msgs;

class AutoToManual : public ::testing::Test, am::AMLifeCycle 
{
public:
  TransitionUtility t;
};

TEST_F(AutoToManual, testState_AutoToManual)
{
  t.sendCommandUntilResponseReceived(OperatorCommand::ARM, t.armed);
  ASSERT_TRUE(t.armed);

  t.sendCommandUntilResponseReceived(OperatorCommand::LAUNCH, t.in_auto);
  ASSERT_TRUE(t.in_auto);
  
  t.sendCommandUntilResponseReceived(OperatorCommand::MANUAL, t.manual);
  ASSERT_TRUE(t.manual);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "auto_to_manual_rostest");

  return RUN_ALL_TESTS();
}