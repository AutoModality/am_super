#include <am_rostest_lib/am_rostest.h>

/**
 * Confirms errors will be ignored during the tolerance duration
 * and that they will be respected after the tolerance duration.
 */
class LifeCycleErrorCofingureToleranceTest : public RostestBase, public am::AMLifeCycle
{

void onConfigure()
{
  //do nothing to block going inactive
}




protected:
  LifeCycleErrorCofingureToleranceTest() : RostestBase() {}
};



TEST_F(LifeCycleErrorCofingureToleranceTest, testError_Tolerance)
{

  //don't report any errors until configuring
  waitUntil(LifeCycleState::CONFIGURING);

  
  ros::Rate loop_rate(1);
  int seconds_configuring =0;
  //report error and wait only half the time for cushion 
  int max_seconds_to_wait = configure_tolerance_s / 2;
  ROS_WARN_STREAM("Reporting error for " << max_seconds_to_wait << " seconds ");
  //first prove that reporting error will be ignored during tolerance
  while (seconds_configuring < max_seconds_to_wait && ros::ok() )
  {
    error("NKO0");
    ros::spinOnce();
    loop_rate.sleep();
    seconds_configuring++;
    //TODO: should be in warning
    ASSERT_EQ(LifeCycleStatus::OK,getStatus());
    ROS_WARN_STREAM("Error is still tolerated at " << seconds_configuring << " seconds ");

  }
  
  while(!nodeStatusReceived(ros::this_node::getName(),LifeCycleStatus::ERROR) && ros::ok())
  {
    error("NAE2");
    ros::spinOnce();
    loop_rate.sleep();
    ROS_WARN_STREAM("Reporting error during state ");
  }

  ASSERT_EQ(LifeCycleStatus::ERROR,getStatus());
  
  waitUntil(LifeCycleState::ERROR_PROCESSING);
  waitUntil(LifeCycleState::FINALIZED);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}