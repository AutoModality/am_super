#include <am_rostest_lib/am_rostest.h>

/** Indicating a forced error should go into error reporting regardless of the state. 
 */
class LifeCycleErrorTest : public RostestBase, public am::AMLifeCycle
{
protected:

  LifeCycleErrorTest() : 
    RostestBase() 
  {}


  void onConfigure()
  {
    //do nothing...stay in configure
  }
};

TEST_F(LifeCycleErrorTest, testStatus_Error)
{
  waitUntil(LifeCycleState::CONFIGURING,"3K3K");
  ASSERT_EQ(LifeCycleStatus::OK,getStatus());
  errorTerminal("forcing error during configuration", "NNAQ");
  waitUntil(LifeCycleState::ERROR_PROCESSING,"NAKW");
  waitUntil(LifeCycleStatus::ERROR,"IUIU");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}