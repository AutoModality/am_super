#include "../rostest_transition.h" //FIXME: currently relative path

using namespace brain_box_msgs;

class AbortToDisarming : public RostestTransition {};

TEST_F(AbortToDisarming, testState_AbortToDisarming)
{
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::ARM, RostestTransition::armed);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::LAUNCH, RostestTransition::in_auto);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::ABORT, RostestTransition::abort);
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::LANDED, RostestTransition::disarming);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());
  return RUN_ALL_TESTS();
}