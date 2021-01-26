#include <am_rostest_lib/am_rostest.h>

class ReadyToShutdown : public RostestBase, am::AMLifeCycle
{
protected:
  ReadyToShutdown() : RostestBase() {}
};

TEST_F(ReadyToShutdown, testState_SuccessfulFlight)
{
  waitUntil(LifeCycleState::CONFIGURING);
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  RostestBase::shutdown(); //since lifeCycle also has a shutdown, need to be specific
  waitUntilMissionState(brain_box_msgs::VxState::SHUTDOWN);
  //the test launch file timeout acts as an assertion if any wait is blocked and unfinishing
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}