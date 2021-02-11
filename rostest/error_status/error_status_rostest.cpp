#include <am_rostest_lib/am_rostest.h>

class LifeCycleErrorTest : public RostestBase, public am::AMLifeCycle
{
protected:
  class TestStat
  {
  public:
    friend class LifeCycleErrorTest;

    AMStat stat1 = AMStat("s1", "Stat 1", 0, 1);


    TestStat(AMStatList &stat_list)
    {
      stat_list.add(&stat1);
    }
  };

  TestStat stats_;

  LifeCycleErrorTest() : 
    stats_(stats_list_),
    RostestBase() 
  {}
};

TEST_F(LifeCycleErrorTest, testStatus_Error)
{
  waitUntil(LifeCycleState::CONFIGURING);
  waitUntil(LifeCycleState::INACTIVE);
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  
  //stat value initially 0, exceed 0 for warn and 1 for error according to TestStat
  stats_.stat1++;
  waitUntilStatus(LifeCycleStatus::WARN);
  stats_.stat1++;
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