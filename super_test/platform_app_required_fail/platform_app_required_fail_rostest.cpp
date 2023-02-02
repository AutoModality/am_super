#include <am_rostest_lib/am_rostest.h>

class PlatformAppRequiredFailTest : public RostestBase, am::AMLifeCycle
{
protected:

  PlatformAppRequiredFailTest() : RostestBase() {
  }

};

TEST_F(PlatformAppRequiredFailTest, requiredPlatformInLaunchFile)
{
  std::string missing = "missing";
  
  std::string platform_required_param;
  ros::param::param<string>("/am_super/platform/required", platform_required_param, missing);
  ASSERT_EQ(platform_required_param,missing);

  std::string platform_app_required_param;
  ros::param::param<string>("/am_super/platform/app/required", platform_app_required_param, missing);
  ASSERT_EQ(platform_app_required_param,"other");
  
  std::string platform_actual_param;
  ros::param::param<string>("/am_super/platform/actual", platform_actual_param, missing);
  ASSERT_EQ(platform_actual_param,"maker_model_test");
  waitUntil(LifeCycleState::CONFIGURING,"LL88");
  waitUntil(LifeCycleState::FINALIZED,"88LL");
  waitUntilMissionState(brain_box_msgs::VxState::SHUTDOWN,"L8L8");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}