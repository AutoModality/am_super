#include <am_rostest_lib/am_rostest.h>

class LifeCycleNodeTest : public RostestBase, am::AMLifeCycle
{
protected:
  LifeCycleNodeTest() : RostestBase() {}
};

TEST_F(LifeCycleNodeTest, testState_SuccessfulFlight)
{
  waitUntil(LifeCycleState::CONFIGURING);
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY,"A3DE");
  arm();
  waitUntil(LifeCycleState::ACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::ARMED,"AZZD");
  launch();
  waitUntilMissionState(brain_box_msgs::VxState::AUTO,"ZZAD");
  landed();
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY,"DDZA");
  //the test launch file timeout acts as an assertion if any wait is blocked and unfinishing
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}