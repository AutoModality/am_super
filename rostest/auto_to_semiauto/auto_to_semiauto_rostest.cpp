#include "../rostest_transition.h" //FIXME: currently relative path

using namespace brain_box_msgs;

class AutoToSemiAuto : public RostestTransition {};

TEST_F(AutoToSemiAuto, testState_AutoToSemiAuto)
{
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::ARM, RostestTransition::armed);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::LAUNCH, RostestTransition::in_auto);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::PAUSE, RostestTransition::semi_auto);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::RESUME, RostestTransition::auto_after_semiauto);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "auto_to_semiauto_rostest");
  return RUN_ALL_TESTS();
}