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
  ASSERT_EQ(expected_success, success);

  if(success)
  {
    LifeCycleStatus actual = mediator.getStatus(info);
    EXPECT_EQ(expected_status, actual) << mediator.statusToString(expected_status) << " not equal to " 
    << mediator.statusToString(actual);
  }
}

TEST(LifeCycleMediator, getAndSetStatus_ALL)
{
  vector<LifeCycleStatus> all = AMLifeCycle::getLifeCycleStatuses();

  for (int i = 0; i < all.size(); i++) //exclude ERROR_PROCESSING
  {
    EXPECT_LIFE_CYCLE_STATUS(all[i], true);
  }
}

void EXPECT_LIFE_CYCLE_STATE(LifeCycleState expected_state,bool expected_success)
{
  AMLifeCycleMediator mediator;
  AMLifeCycleMediator::LifeCycleInfo info;
  bool success = mediator.setState(expected_state, info);
  EXPECT_EQ(expected_success, success);

  if(success)
  {
    LifeCycleState actual = mediator.getState(info);
    EXPECT_EQ(expected_state, actual) << mediator.stateToString(expected_state) << " not equal to " 
    << mediator.stateToString(actual);
  }
}

TEST(LifeCycleMediator, getAndSetState_ALL)
{
  vector<LifeCycleState> all = AMLifeCycleMediator::getLifeCycleStates();

  for (int i = 0; i < all.size(); i++) //exclude ERROR_PROCESSING
  {
    EXPECT_LIFE_CYCLE_STATE(all[i], true);
  }
}

TEST(LifeCycleMediator, commandTestStringConversion)
{
  vector<LifeCycleCommand> allCommands = AMLifeCycleMediator::getLifeCycleCommands();
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
  vector<LifeCycleStatus> allStatus = AMLifeCycleMediator::getLifeCycleStatuses();
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
  vector<LifeCycleState> allStates = AMLifeCycleMediator::getLifeCycleStates();
  string str;
  LifeCycleState state;

  for(int i = 0; i < allStates.size(); i++) 
  {
    str = AMLifeCycleMediator::stateToString(allStates[i]);
    EXPECT_TRUE(AMLifeCycleMediator::stringToState(str, state));
    EXPECT_EQ(allStates[i], state);
  }
}

TEST(LifeCycleMediator, statusToString_BadStatusReturnsInvalidString)
{
  
}