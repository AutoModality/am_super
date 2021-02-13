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

  /** stats will not pass until a sample is received. but node will not go status::ERROR
   * until configuration tolerance times out, which will cause us to enter ERROR_PROCESSING 
   * and setstatus to ERROR, resulting in shutting down */
  stats_.stat1 = 0;
  waitUntil(LifeCycleStatus::OK,"REDF");
  waitUntil(LifeCycleState::INACTIVE,"NAQP");
  
  //exceed 0 for warn and 1 for error according to TestStat
  stats_.stat1++;
  waitUntil(LifeCycleStatus::WARN,"QNAI");
  //still inactive...
  waitUntil(LifeCycleState::INACTIVE,"HEI3");

  stats_.stat1++;

  //wait for tolerance to time out, then catch error
  waitUntil(LifeCycleStatus::ERROR,"QAQA");
  waitUntil(LifeCycleState::ERROR_PROCESSING,"AQ90");
  
  waitUntilMissionState(brain_box_msgs::VxState::SHUTDOWN);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}