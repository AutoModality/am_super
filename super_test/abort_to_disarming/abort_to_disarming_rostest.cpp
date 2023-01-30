#include <super_lib/am_super_test.h> 

class AbortToDisarming : public AMSuperTest
{
protected:
  AbortToDisarming() : AMSuperTest("abort_to_disarming") {}
};

TEST_F(AbortToDisarming, testState_AbortToDisarming)
{
  waitUntilMissionState(brain_box_msgs::msg::VxState::READY,"N3DJ");
  arm();
  waitUntilMissionState(brain_box_msgs::msg::VxState::ARMED,"XX3X");
  launch();
  waitUntilMissionState(brain_box_msgs::msg::VxState::AUTO,"YYUI");
  abort();
  waitUntilMissionState(brain_box_msgs::msg::VxState::ABORT,"NSKE");
  landed();
  waitUntilMissionState(brain_box_msgs::msg::VxState::DISARMING,"XXCV");
}

int main(int argc, char** argv)
{
  std::cout << "STARTING ROS" << std::endl;
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}