#include <am_rostest_lib/am_rostest.h>

class LifeCycleErrorTestWithoutStats : public RostestBase, public am::AMLifeCycle
{
protected:
  LifeCycleErrorTestWithoutStats() : RostestBase() 
  {}
};

TEST_F(LifeCycleErrorTestWithoutStats, testStatus_Error)
{
  waitUntil(LifeCycleState::CONFIGURING);
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  
  error("HEOL");
  waitUntil(LifeCycleStatus::ERROR,"QNA0");
  waitUntil(LifeCycleState::ERROR_PROCESSING);
  
  waitUntilMissionState(brain_box_msgs::VxState::SHUTDOWN);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}