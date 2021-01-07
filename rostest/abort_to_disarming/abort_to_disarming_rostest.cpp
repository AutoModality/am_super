#include <am_rostest_lib/am_rostest.h> 

class AbortToDisarming : public RostestBase, am::AMLifeCycle
{
protected:
  AbortToDisarming() : RostestBase(ros::this_node::getName()) {}
};

TEST_F(AbortToDisarming, testState_AbortToDisarming)
{
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  arm();
  waitUntilMissionState(brain_box_msgs::VxState::ARMED);
  launch();
  waitUntilMissionState(brain_box_msgs::VxState::AUTO);
  abort();
  waitUntilMissionState(brain_box_msgs::VxState::ABORT);
  landed();
  waitUntilMissionState(brain_box_msgs::VxState::DISARMING);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());
  return RUN_ALL_TESTS();
}