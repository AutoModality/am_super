#include <am_rostest_lib/am_rostest.h>

class LifeCycleErrorTest : public RostestBase, public am::AMLifeCycle
{
protected:
  LifeCycleErrorTest() : RostestBase(ros::this_node::getName()) {}
};

TEST_F(LifeCycleErrorTest, testStatus_Error)
{
  waitUntil(LifeCycleState::CONFIGURING);
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY);

  //simulate an error in LifeCycle
  am::AMLifeCycle::error();

  //LifeCycle State and Status should go error for this node
  waitUntilStatus(LifeCycleStatus::ERROR);
  waitUntil(LifeCycleState::ERROR_PROCESSING);
  waitUntilMissionState(brain_box_msgs::VxState::SHUTDOWN);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}