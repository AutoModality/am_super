#include "../rostest_transition.h" //FIXME: currently relative path

using namespace brain_box_msgs;

class ArmedToReady : public RostestTransition {};

TEST_F(ArmedToReady, testState_ArmedToReady)
{
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::ARM, RostestTransition::armed);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::CANCEL, RostestTransition::disarming);
  RostestTransition::waitUntilResponseReceived(RostestTransition::ready_after_armed);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "armed_to_ready_rostest");
  return RUN_ALL_TESTS();
}