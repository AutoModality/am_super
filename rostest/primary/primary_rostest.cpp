#include <super_rostest_lib/rostest_base.h>

class LifeCycleNodeTest : public RostestBase, am::AMLifeCycle
{
protected:
  LifeCycleNodeTest() : RostestBase(ros::this_node::getName()) {}
};

TEST_F(LifeCycleNodeTest, testState_SuccessfulFlight)
{
  waitUntil(LifeCycleState::CONFIGURING);
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  arm();
  waitUntil(LifeCycleState::ACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::ARMED);
  launch();
  waitUntilMissionState(brain_box_msgs::VxState::AUTO);
  landed();
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  //the test launch file timeout acts as an assertion if any wait is blocked and unfinishing
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}