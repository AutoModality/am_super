#include <gtest/gtest.h>
#include <super_lib/am_life_cycle_mediator.h>
#include <super_lib/am_life_cycle.h>

using namespace am;
using namespace std;



void ASSERT_LIFE_CYCLE_STATUS(LifeCycleStatus expected_status,bool expected_success)
{
  AMLifeCycleMediator mediator;
  AMLifeCycleMediator::LifeCycleInfo info;
  bool success = mediator.setStatus(expected_status, info);
  LifeCycleStatus actual = mediator.getStatus(info);
  EXPECT_EQ(actual, expected_status);
  EXPECT_EQ(success,expected_success);
}

TEST(LifeCycleMediator, getAndSetStatus_OK)
{
  ASSERT_LIFE_CYCLE_STATUS(LifeCycleStatus::OK, true);
}

TEST(LifeCycleMediator, getAndSetStatus_WARN)
{
  ASSERT_LIFE_CYCLE_STATUS(LifeCycleStatus::WARN, true);
}

TEST(LifeCycleMediator, getAndSetStatus_ERROR)
{
  ASSERT_LIFE_CYCLE_STATUS(LifeCycleStatus::ERROR, true);
}

TEST(LifeCycleMediator, getAndSetStatus_ignores_LAST_STATUS)
{
  AMLifeCycleMediator mediator;
  AMLifeCycleMediator::LifeCycleInfo info;
  
  bool success = mediator.setStatus(LifeCycleStatus::OK, info);
  ASSERT_TRUE(success); //following tests depend on this being true, therefore assert

  success = mediator.setStatus(LifeCycleStatus::LAST_STATUS, info);
  LifeCycleStatus final_status = mediator.getStatus(info);

  EXPECT_FALSE(success); //expect setStatus to return false;
  EXPECT_EQ(LifeCycleStatus::OK, final_status); //expect no change in status
}

void ASSERT_LIFE_CYCLE_STATE(LifeCycleState expected_state,bool expected_success)
{
  AMLifeCycleMediator mediator;
  AMLifeCycleMediator::LifeCycleInfo info;
  bool success = mediator.setState(expected_state, info);
  LifeCycleState actual = mediator.getState(info);
  EXPECT_EQ(actual, expected_state);
  EXPECT_EQ(success,expected_success);
}

TEST(LifeCycleMediator, getAndSetState_all_except_ERROR_PROCESSING)
{
  vector<LifeCycleState> all = AMLifeCycle::getLifeCycleStates();

  for (int i = 0; i < all.size() - 1; i++) //exclude ERROR_PROCESSING
  {
    ASSERT_LIFE_CYCLE_STATE(all[i], true);
  }
}

TEST(LifeCycleMediator, getAndSetState_ERROR_PROCESSING)
{
  ASSERT_LIFE_CYCLE_STATE(LifeCycleState::ERROR_PROCESSING, true);
}

TEST(LifeCycleMediator, commandTestStringConversion)
{
  vector<LifeCycleCommand> allCommands = AMLifeCycle::getLifeCycleCommands();
  string str;
  LifeCycleCommand cmd;

  for(int i = 0; i < allCommands.size(); i++)
  {
    str = AMLifeCycleMediator::commandToString(allCommands[i]);
    EXPECT_TRUE(AMLifeCycleMediator::stringToCommand(str, cmd));
    EXPECT_EQ(allCommands[i], cmd);
  }
}