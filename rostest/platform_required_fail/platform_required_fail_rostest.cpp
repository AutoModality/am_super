#include <am_rostest_lib/am_rostest.h>

class PlatformRequiredFailTest : public RostestBase, am::AMLifeCycle
{
protected:

  PlatformRequiredFailTest() : RostestBase() {
  }

};

TEST_F(PlatformRequiredFailTest, requiredPlatformDifferentThanActual)
{
  std::string platform_required_param;
  ros::param::param<string>("/am_super/platform/required", platform_required_param, "missing");
  ASSERT_EQ(platform_required_param,"test");
  
  std::string platform_actual_param;
  ros::param::param<string>("/am_super/platform/actual", platform_actual_param, "missing");
  ASSERT_EQ(platform_actual_param,"not_test");
  
  waitUntil(LifeCycleState::CONFIGURING,"UIYT");
  waitUntil(LifeCycleState::FINALIZED,"23SS");
  waitUntilMissionState(brain_box_msgs::VxState::SHUTDOWN,"HYUJ");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}