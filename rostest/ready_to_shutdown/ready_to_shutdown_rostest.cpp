#include "../rostest_transition.h" //FIXME: currently relative path

using namespace brain_box_msgs;

class ReadyToShutdown : public RostestTransition {};

TEST_F(ReadyToShutdown, testState_ReadyToShutdown)
{
  RostestTransition::sendCommandUntilResponseReceived(OperatorCommand::SHUTDOWN, RostestTransition::shutdown);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());
  return RUN_ALL_TESTS();
}