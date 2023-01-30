#include <am_rostest_lib/am_rostest.h> 

class ArmedToReady : public RostestBase, am::AMLifeCycle
{
protected:
  ArmedToReady() : RostestBase() {}
};

TEST_F(ArmedToReady, testState_ArmedToReady)
{
  waitUntilMissionState(brain_box_msgs::VxState::READY,"SNWS");
  arm();
  waitUntilMissionState(brain_box_msgs::VxState::ARMED,"AXAD");
  cancel();
  waitUntilMissionState(brain_box_msgs::VxState::DISARMING,"ANZD");
  waitUntilMissionState(brain_box_msgs::VxState::READY,"QADE");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "armed_to_ready_rostest");
  return RUN_ALL_TESTS();
}