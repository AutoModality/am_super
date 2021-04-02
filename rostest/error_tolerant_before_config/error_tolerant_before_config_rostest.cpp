#include <am_rostest_lib/am_rostest.h>

/** Shows an error that happens upon construction will be reported as an ERROR during configuration. 
 */
class LifeCycleErrorTest : public RostestBase, public am::AMLifeCycle
{
protected:

  LifeCycleErrorTest() : 
    RostestBase() 
  {
      errorTolerant("tolerant error during construction", "HW8S");
  }
};

TEST_F(LifeCycleErrorTest, testStatus_Error)
{
  waitUntil(LifeCycleState::INACTIVE,"NAKW");
  waitUntil(LifeCycleStatus::OK,"NQNE");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}