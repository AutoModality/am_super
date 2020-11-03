#include "../rostest_transition.h" //FIXME: currently relative path

using namespace brain_box_msgs;

class AutoToAbort : public RostestTransition {};

TEST_F(AutoToAbort, testState_AutoToAbort)
{
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::ARM, RostestTransition::armed);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::LAUNCH, RostestTransition::in_auto);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::ABORT, RostestTransition::abort);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "auto_to_semiauto_rostest");
  return RUN_ALL_TESTS();
}