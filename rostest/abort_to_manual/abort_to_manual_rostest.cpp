#include <am_rostest_lib/am_rostest.h> 

class AbortToManual : public RostestBase, am::AMLifeCycle
{
protected:
  AbortToManual() : RostestBase() {}
};

TEST_F(AbortToManual, testState_AbortToDisarming)
{
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  arm();
  waitUntilMissionState(brain_box_msgs::VxState::ARMED);
  launch();
  waitUntilMissionState(brain_box_msgs::VxState::AUTO);
  abort();
  waitUntilMissionState(brain_box_msgs::VxState::ABORT);
  manual();
  waitUntilMissionState(brain_box_msgs::VxState::MANUAL);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());
  return RUN_ALL_TESTS();
}