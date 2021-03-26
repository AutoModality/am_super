#include <am_rostest_lib/am_rostest.h>

class PlatformRequiredPassTest : public RostestBase, am::AMLifeCycle
{
protected:

  PlatformRequiredPassTest() : RostestBase() {
  }

};

TEST_F(PlatformRequiredPassTest, requiredPlatformInLaunchFile)
{
  std::string platform_required_param;
  ros::param::param<string>("/am_super/platform/required", platform_required_param, "missing");
  ASSERT_EQ(platform_required_param,"test");
  
  std::string platform_actual_param;
  ros::param::param<string>("/am_super/platform/actual", platform_actual_param, "missing");
  ASSERT_EQ(platform_actual_param,"test");
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