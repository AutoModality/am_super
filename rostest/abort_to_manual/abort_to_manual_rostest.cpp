#include "../rostest_transition.h" //FIXME: currently relative path

using namespace brain_box_msgs;

class AbortToManual : public RostestTransition {};

TEST_F(AbortToManual, testState_AbortToManual)
{
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::ARM, RostestTransition::armed);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::LAUNCH, RostestTransition::in_auto);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::ABORT, RostestTransition::abort);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::MANUAL, RostestTransition::manual);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "auto_to_semiauto_rostest");
  return RUN_ALL_TESTS();
}