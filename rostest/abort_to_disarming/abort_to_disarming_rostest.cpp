#include <am_rostest_lib/am_rostest.h> 

class AbortToDisarming : public RostestBase, am::AMLifeCycle
{
protected:
  AbortToDisarming() : RostestBase(ros::this_node::getName()) {}
};

/* 
  BUG: floods output because of another bug:
  if one of the nodes in the manifest fails for some reason, super responds by resending lifecyclecommands
  until the node is in the proper state, but for some reason, supers sends lifecyclecommands to all manifested 
  nodes, as if they are all failing the check. Note that super itself is a manifested node. For transitioning ABORT->DISARMING, 
  we transition when we receive MC:Complete. The test is simply waiting for a response from super that it has successfully
  transitioned into DISARMING, then it passes, but since it takes time for the response to reach this test node, super continues
  with its behavior. Super remains in DISARMING until all manifested nodes are in the correct lifecyclestate; for everyone besides super, 
  that would be INACTIVE since DISARMING->READY requires nodes to be INACTIVE. Since this test node will be in ACTIVE, we fail the 
  check and will be sent DEACTIVATE, but since the bug exists, super will also be sent DEACTIVATE. Right now, super transitions
  into ACTIVE when it becomes INACTIVE, so a loop occurs where super's lifecycle cycles between ACTIVE->INACTIVE->ACTIVE... and
  the output is flooded with the transitions. The loop ends when BOTH nodes lifecycle becomes INACTIVE.

  FIXME: only transition super's lifecycle to ACTIVE when its lifecycle is INACTIVE and its system state is in READY. This will
  fix the output flooding while in system state DISARMING. The main bug causing this all is still unknown. 
*/
TEST_F(AbortToDisarming, testState_AbortToDisarming)
{
  waitUntilMissionState(brain_box_msgs::VxState::READY);
  arm();
  waitUntilMissionState(brain_box_msgs::VxState::ARMED);
  launch();
  waitUntilMissionState(brain_box_msgs::VxState::AUTO);
  abort();
  waitUntilMissionState(brain_box_msgs::VxState::ABORT);
  landed();
  waitUntilMissionState(brain_box_msgs::VxState::DISARMING);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());
  return RUN_ALL_TESTS();
}