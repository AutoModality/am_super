#include <am_rostest_lib/am_rostest.h> 

class AbortToDisarming : public RostestBase, am::AMLifeCycle
{
protected:
  AbortToDisarming() : RostestBase() {}
};

TEST_F(AbortToDisarming, testState_AbortToDisarming)
{
  waitUntilMissionState(brain_box_msgs::VxState::READY,"N3DJ");
  arm();
  waitUntilMissionState(brain_box_msgs::VxState::ARMED,"XX3X");
  launch();
  waitUntilMissionState(brain_box_msgs::VxState::AUTO,"YYUI");
  abort();
  waitUntilMissionState(brain_box_msgs::VxState::ABORT,"NSKE");
  landed();
  waitUntilMissionState(brain_box_msgs::VxState::DISARMING,"XXCV");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());
  return RUN_ALL_TESTS();
}