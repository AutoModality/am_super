#include <am_rostest_lib/am_rostest.h>

class PlatformRequiredPassTest : public RostestBase, am::AMLifeCycle
{
protected:

  PlatformRequiredPassTest() : RostestBase() {
  }

};

TEST_F(PlatformRequiredPassTest, requiredPlatformInLaunchFile)
{
 waitUntil(LifeCycleState::CONFIGURING,"NWO2");
 waitUntil(LifeCycleState::INACTIVE,"NN29");
 waitUntilMissionState(brain_box_msgs::VxState::READY,"89UI");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}