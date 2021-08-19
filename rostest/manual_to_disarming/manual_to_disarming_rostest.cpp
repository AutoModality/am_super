#include <am_rostest_lib/am_rostest.h>

class ManualToDisarming : public RostestBase, am::AMLifeCycle
{
protected:
  ManualToDisarming() : RostestBase() {}
};

TEST_F(ManualToDisarming, testState_SuccessfulFlight)
{
  waitUntil(LifeCycleState::CONFIGURING);
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY,"NSK3");
  arm();
  waitUntil(LifeCycleState::ACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::ARMED,"38JK");
  launch();
  waitUntilMissionState(brain_box_msgs::VxState::AUTO,"22K3");
  manual();
  waitUntilMissionState(brain_box_msgs::VxState::MANUAL,"21KS");
  landed();
  waitUntilMissionState(brain_box_msgs::VxState::DISARMING,"2NSK");
  //the test launch file timeout acts as an assertion if any wait is blocked and unfinishing
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}