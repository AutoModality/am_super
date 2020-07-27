#include <gtest/gtest.h>  // googletest header file
#include <super_lib/am_life_cycle.h>

using namespace am;
using namespace std;

TEST(LifeCycle, statusToString_BadStatusReturnsInvalidString)
{
  LifeCycleStatus status = LifeCycleStatus(20);
  string str;
  str = AMLifeCycle::statusToString(status);
  ASSERT_EQ(str, AMLifeCycle::EMPTY_STRING);
}

TEST(LifeCycle, statusTestStringConversion)
{
  vector<LifeCycleStatus> allStatus = AMLifeCycle::getLifeCycleStatus();
  string str;
  LifeCycleStatus status;

  for(int i = 0; i < allStatus.size(); i++)
  {
    str = AMLifeCycle::statusToString(allStatus[i]);
    ROS_INFO_STREAM(str << '\n');
    ASSERT_TRUE(AMLifeCycle::stringToStatus(str, status));
    ASSERT_EQ(allStatus[i], status);
  }
} 

TEST(LifeCycle, stringToStatus_BadStringReturnsFalse)
{
  LifeCycleStatus initial_status = LifeCycleStatus::OK;
  LifeCycleStatus status = initial_status;
  string bad_string;
  bool bad_strings_should_return_false = AMLifeCycle::stringToStatus(bad_string, status);
  ASSERT_FALSE(bad_strings_should_return_false);
  ASSERT_EQ(status, initial_status) << "Status should be unchanged since the string is bad";
}

TEST(LifeCycle, commandToString_BadCommandReturnsInvalidString)
{
  LifeCycleCommand cmd = LifeCycleCommand(20);
  string str;
  str = AMLifeCycle::commandToString(cmd);
  ASSERT_EQ(str, AMLifeCycle::EMPTY_STRING);
}

TEST(LifeCycle, commandTestStringConversion)
{
  vector<LifeCycleCommand> allCommands = AMLifeCycle::getLifeCycleCommands();
  string str;
  LifeCycleCommand cmd;

  for(int i = 0; i < allCommands.size(); i++)
  {
    str = AMLifeCycle::commandToString(allCommands[i]);
    ASSERT_TRUE(AMLifeCycle::stringToCommand(str, cmd));
    ASSERT_EQ(allCommands[i], cmd);
  }
}

TEST(LifeCycle, stringToCommand_BadStringReturnsFalse)
{
  LifeCycleCommand initial_cmd = LifeCycleCommand::ACTIVATE;
  LifeCycleCommand cmd = initial_cmd;
  string bad_string;
  bool bad_strings_should_return_false = AMLifeCycle::stringToCommand(bad_string, cmd);
  ASSERT_FALSE(bad_strings_should_return_false);
  ASSERT_EQ(cmd, initial_cmd) << "Command should be unchanged since the string is bad";
}

TEST(LifeCycle, stateToString_BadStateReturnsInvalidString)
{
  LifeCycleState state = LifeCycleState(20);
  string str;
  str = AMLifeCycle::stateToString(state);
  ASSERT_EQ(str, AMLifeCycle::STATE_INVALID_STRING);
}

TEST(LifeCycle, stateTestStringConversion)
{
  vector<LifeCycleState> allStates = AMLifeCycle::getLifeCycleStates();
  string str;
  LifeCycleState state;

  for(int i = 0; i < allStates.size(); i++) 
  {
    str = AMLifeCycle::stateToString(allStates[i]);
    ASSERT_TRUE(AMLifeCycle::stringToState(str, state));
    ASSERT_EQ(allStates[i], state);
  }
}

TEST(LifeCycle, stringToState_BadStringReturnsFalse)
{
  LifeCycleState default_state=LifeCycleState::FINALIZED;
  LifeCycleState state = default_state;
  string bad_string;
  bool bad_strings_should_return_false = AMLifeCycle::stringToState(bad_string,state);
  ASSERT_FALSE(bad_strings_should_return_false);
  ASSERT_EQ(state,default_state) << "State should be unchanged since the string is bad";
}