#include <am_rostest_lib/am_rostest.h>

class AutoToManual : public RostestBase, am::AMLifeCycle
{
protected:
  AutoToManual() : RostestBase() {}
};

TEST_F(AutoToManual, testState_SuccessfulFlight)
{
  waitUntil(LifeCycleState::CONFIGURING);
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY,"LLLE");
  arm();
  waitUntil(LifeCycleState::ACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::ARMED,"PWDK");
  launch();
  waitUntilMissionState(brain_box_msgs::VxState::AUTO,"ZDWD");
  manual();
  waitUntilMissionState(brain_box_msgs::VxState::MANUAL,"ZAZD");
  //the test launch file timeout acts as an assertion if any wait is blocked and unfinishing
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}