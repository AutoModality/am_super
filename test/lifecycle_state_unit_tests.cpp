#include <gtest/gtest.h>  // googletest header file
#include "gmock/gmock.h" 
#include <super_lib/am_life_cycle.h>

using namespace am;
using namespace std;


class A
{
protected:
  string yourEyesOnly()
  {
    return "from A";
  }
};

class LifeCycleTestFixture : public ::testing::Test, public :: A
{
protected:
  string see = "me";
  string someMethod()
  {
    return "too";
  }
};


TEST_F(LifeCycleTestFixture, setStatus_AllAllPersistent)
{
  EXPECT_EQ(see,"you");
  EXPECT_EQ(someMethod(),"jeez");
  EXPECT_EQ(yourEyesOnly(),"bee");
}

TEST(LifeCycle, statusToString_BadStatusReturnsInvalidString)
{
  LifeCycleStatus status = LifeCycleStatus(20);
  string str;
  str = AMLifeCycle::statusToString(status);
  EXPECT_EQ(str, AMLifeCycle::EMPTY_STRING);
}

TEST(LifeCycle, getLifeCycleStatuses_AllReturnedInOrder)
{
  vector<LifeCycleStatus> all = AMLifeCycle::getLifeCycleStatuses();
  EXPECT_EQ(all[0],LifeCycleStatus::OK);
  EXPECT_EQ(all[1],LifeCycleStatus::WARN);
  EXPECT_EQ(all[2],LifeCycleStatus::ERROR);
}

TEST(LifeCycle, statusTestStringConversion)
{
  vector<LifeCycleStatus> allStatus = AMLifeCycle::getLifeCycleStatuses();
  string string_from_status;
  LifeCycleStatus status_from_string;

  for(LifeCycleStatus expected_status: allStatus)
  {
    string_from_status = AMLifeCycle::statusToString(expected_status);
    bool success = AMLifeCycle::stringToStatus(string_from_status, status_from_string);
    EXPECT_EQ(status_from_string,expected_status);
    EXPECT_TRUE(success);
  }
} 

TEST(LifeCycle, stringToStatus_BadStringReturnsFalse)
{
  LifeCycleStatus initial_status = LifeCycleStatus::OK;
  LifeCycleStatus status = initial_status;
  string bad_string;
  bool bad_strings_should_return_false = AMLifeCycle::stringToStatus(bad_string, status);
  EXPECT_FALSE(bad_strings_should_return_false);
  EXPECT_EQ(status, initial_status) << "Status should be unchanged since the string is bad";
}

TEST(LifeCycle, commandToString_BadCommandReturnsInvalidString)
{
  LifeCycleCommand cmd = LifeCycleCommand(20);
  string str;
  str = AMLifeCycle::commandToString(cmd);
  EXPECT_EQ(str, AMLifeCycle::EMPTY_STRING);
}

TEST(LifeCycle, commandTestStringConversion)
{
  vector<LifeCycleCommand> allCommands = AMLifeCycle::getLifeCycleCommands();
  string str;
  LifeCycleCommand cmd;

  for(int i = 0; i < allCommands.size(); i++)
  {
    str = AMLifeCycle::commandToString(allCommands[i]);
    EXPECT_TRUE(AMLifeCycle::stringToCommand(str, cmd));
    EXPECT_EQ(allCommands[i], cmd);
  }
}

TEST(LifeCycle, stringToCommand_BadStringReturnsFalse)
{
  LifeCycleCommand initial_cmd = LifeCycleCommand::ACTIVATE;
  LifeCycleCommand cmd = initial_cmd;
  string bad_string;
  bool bad_strings_should_return_false = AMLifeCycle::stringToCommand(bad_string, cmd);
  EXPECT_FALSE(bad_strings_should_return_false);
  EXPECT_EQ(cmd, initial_cmd) << "Command should be unchanged since the string is bad";
}

TEST(LifeCycle, stateToString_BadStateReturnsInvalidString)
{
  LifeCycleState state = LifeCycleState(20);
  string str;
  str = AMLifeCycle::stateToString(state);
  EXPECT_EQ(str, AMLifeCycle::STATE_INVALID_STRING);
}

TEST(LifeCycle, stateTestStringConversion)
{
  vector<LifeCycleState> allStates = AMLifeCycle::getLifeCycleStates();
  string str;
  LifeCycleState state;

  for(int i = 0; i < allStates.size(); i++) 
  {
    str = AMLifeCycle::stateToString(allStates[i]);
    EXPECT_TRUE(AMLifeCycle::stringToState(str, state));
    EXPECT_EQ(allStates[i], state);
  }
}

TEST(LifeCycle, stringToState_BadStringReturnsFalse)
{
  LifeCycleState default_state=LifeCycleState::FINALIZED;
  LifeCycleState state = default_state;
  string bad_string;
  bool bad_strings_should_return_false = AMLifeCycle::stringToState(bad_string,state);
  EXPECT_FALSE(bad_strings_should_return_false);
  EXPECT_EQ(state,default_state) << "State should be unchanged since the string is bad";
}