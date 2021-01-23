#include <am_rostest_lib/am_rostest.h>

class AutoToSemiAuto : public RostestBase, am::AMLifeCycle
{
protected:
  AutoToSemiAuto() : RostestBase() {}
};

TEST_F(AutoToSemiAuto, testState_SuccessfulFlight)
{
  waitUntil(LifeCycleState::CONFIGURING);
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  arm();
  waitUntil(LifeCycleState::ACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::ARMED);
  launch();
  waitUntilMissionState(brain_box_msgs::VxState::AUTO);
  pause();
  waitUntilMissionState(brain_box_msgs::VxState::SEMI_AUTO);
  //the test launch file timeout acts as an assertion if any wait is blocked and unfinishing
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}