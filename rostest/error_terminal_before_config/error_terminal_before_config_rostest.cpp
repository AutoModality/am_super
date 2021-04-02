#include <am_rostest_lib/am_rostest.h>

/** Shows an error that happens upon construction will be reported as an ERROR during configuration. 
 */
class LifeCycleErrorTest : public RostestBase, public am::AMLifeCycle
{
protected:

  LifeCycleErrorTest() : 
    RostestBase() 
  {
      errorTerminal("forcing error during construction", "HW8S");
  }


  void onConfigure()
  {
    //do nothing...stay in configure
  }
};

TEST_F(LifeCycleErrorTest, testStatus_Error)
{
  ASSERT_EQ(LifeCycleState::ERROR_PROCESSING, getState());
  waitUntil(LifeCycleState::ERROR_PROCESSING,"NAKW");
  waitUntil(LifeCycleStatus::ERROR,"IUIU");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}