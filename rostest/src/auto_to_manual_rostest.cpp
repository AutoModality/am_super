#include <am_super/transition_utility.h>

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