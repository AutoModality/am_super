#include "../rostest_transition.h" //FIXME: currently relative path

using namespace brain_box_msgs;

class AutoToManual : public RostestTransition {};

TEST_F(AutoToManual, testState_AutoToManual)
{
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::ARM, RostestTransition::armed);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::LAUNCH, RostestTransition::in_auto);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::MANUAL, RostestTransition::manual);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "auto_to_manual_rostest");
  return RUN_ALL_TESTS();
}