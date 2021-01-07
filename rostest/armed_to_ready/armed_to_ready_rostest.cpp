#include <am_rostest_lib/am_rostest.h> 

class ArmedToReady : public RostestBase, am::AMLifeCycle
{
protected:
  ArmedToReady() : RostestBase(ros::this_node::getName()) {}
};

TEST_F(ArmedToReady, testState_ArmedToReady)
{
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  arm();
  waitUntilMissionState(brain_box_msgs::VxState::ARMED);
  cancel();
  waitUntilMissionState(brain_box_msgs::VxState::READY);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "armed_to_ready_rostest");
  return RUN_ALL_TESTS();
}