#include <am_rostest_lib/am_rostest.h>

class PlatformRequiredFailTest : public RostestBase, am::AMLifeCycle
{
protected:

  PlatformRequiredFailTest() : RostestBase() {
  }

};

TEST_F(PlatformRequiredFailTest, requiredPlatformInLaunchFile)
{
 waitUntil(LifeCycleState::CONFIGURING,"UIYT");
 waitUntil(LifeCycleState::ERROR_PROCESSING,"23SS");
 waitUntilMissionState(brain_box_msgs::VxState::SHUTDOWN,"HYUJ");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}