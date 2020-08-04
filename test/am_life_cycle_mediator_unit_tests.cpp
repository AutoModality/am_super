#include <gtest/gtest.h>
#include <super_lib/am_life_cycle_mediator.h>
#include <super_lib/am_life_cycle.h>

using namespace am;
using namespace std;



void EXPECT_LIFE_CYCLE_STATUS(LifeCycleStatus expected_status,bool expected_success)
{
  AMLifeCycleMediator mediator;
  AMLifeCycleMediator::LifeCycleInfo info;
  bool success = mediator.setStatus(expected_status, info);
  LifeCycleStatus actual = mediator.getStatus(info);
  EXPECT_EQ(expected_status, actual) << mediator.statusToString(expected_status) << " not equal to " 
    << mediator.statusToString(actual);
  EXPECT_EQ(expected_success, success);
}

TEST(LifeCycleMediator, getAndSetStatus_OK)
{
  EXPECT_LIFE_CYCLE_STATUS(LifeCycleStatus::OK, true);
}

TEST(LifeCycleMediator, getAndSetStatus_WARN)
{
  EXPECT_LIFE_CYCLE_STATUS(LifeCycleStatus::WARN, true);
}

TEST(LifeCycleMediator, getAndSetStatus_ERROR)
{
  EXPECT_LIFE_CYCLE_STATUS(LifeCycleStatus::ERROR, true);
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

TEST(LifeCycleMediator, getAndSetState_ignores_LAST_STATUS)
{
  AMLifeCycleMediator mediator;
  AMLifeCycleMediator::LifeCycleInfo info;
  
  bool success = mediator.setState(LifeCycleState::ACTIVE, info);
  ASSERT_TRUE(success); //following tests depend on this being true, therefore assert

  success = mediator.setState(LifeCycleState::LAST_STATE, info);
  LifeCycleState final_status = mediator.getState(info);

  EXPECT_FALSE(success); //expect setStatus to return false;
  EXPECT_EQ(LifeCycleState::ACTIVE, final_status); //expect no change in status
}

void EXPECT_LIFE_CYCLE_STATE(LifeCycleState expected_state,bool expected_success)
{
  AMLifeCycleMediator mediator;
  AMLifeCycleMediator::LifeCycleInfo info;
  bool success = mediator.setState(expected_state, info);
  LifeCycleState actual = mediator.getState(info);
  EXPECT_EQ(expected_state, actual) << mediator.stateToString(expected_state) << " not equal to " 
    << mediator.stateToString(actual);
  EXPECT_EQ(expected_success, success);
}

TEST(LifeCycleMediator, getAndSetState_all_except_ERROR_PROCESSING)
{
  vector<LifeCycleState> all = AMLifeCycle::getLifeCycleStates();

  for (int i = 0; i < all.size() - 1; i++) //exclude ERROR_PROCESSING
  {
    EXPECT_LIFE_CYCLE_STATE(all[i], true);
  }
}

TEST(LifeCycleMediator, getAndSetState_ERROR_PROCESSING)
{
  EXPECT_LIFE_CYCLE_STATE(LifeCycleState::ERROR_PROCESSING, true);
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

TEST(LifeCycleMediator, statusTestStringConversion)
{
  vector<LifeCycleStatus> allStatus = AMLifeCycle::getLifeCycleStatuses();
  string string_from_status;
  LifeCycleStatus status_from_string;

  for(LifeCycleStatus expected_status: allStatus)
  {
    string_from_status = AMLifeCycleMediator::statusToString(expected_status);
    bool success = AMLifeCycleMediator::stringToStatus(string_from_status, status_from_string);
    EXPECT_EQ(status_from_string,expected_status);
    EXPECT_TRUE(success);
  }
} 

TEST(LifeCycleMediator, stateTestStringConversion)
{
  vector<LifeCycleState> allStates = AMLifeCycle::getLifeCycleStates();
  string str;
  LifeCycleState state;

  for(int i = 0; i < allStates.size(); i++) 
  {
    str = AMLifeCycleMediator::stateToString(allStates[i]);
    EXPECT_TRUE(AMLifeCycleMediator::stringToState(str, state));
    EXPECT_EQ(allStates[i], state);
  }
}