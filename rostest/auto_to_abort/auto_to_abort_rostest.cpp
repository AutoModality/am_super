#include <am_rostest_lib/am_rostest.h>

class AutoToAbort : public RostestBase, am::AMLifeCycle
{
protected:
  AutoToAbort() : RostestBase(ros::this_node::getName()) {}
};

TEST_F(AutoToAbort, testState_SuccessfulFlight)
{
  waitUntil(LifeCycleState::CONFIGURING);
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  arm();
  waitUntil(LifeCycleState::ACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::ARMED);
  launch();
  waitUntilMissionState(brain_box_msgs::VxState::AUTO);
  abort();
  waitUntilMissionState(brain_box_msgs::VxState::ABORT);
  //the test launch file timeout acts as an assertion if any wait is blocked and unfinishing
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}