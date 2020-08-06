#include <gtest/gtest.h>
#include <super_lib/am_life_cycle_mediator.h>
#include <super_lib/am_life_cycle.h>

using namespace am;
using namespace std;

AMLifeCycleMediator life_cycle_mediator_;

void EXPECT_LIFE_CYCLE_STATUS(LifeCycleStatus expected_status,bool expected_success)
{
  AMLifeCycleMediator::LifeCycleInfo info;
  bool success = life_cycle_mediator_.setStatus(expected_status, info);
  ASSERT_EQ(expected_success, success);

  if(success)
  {
    LifeCycleStatus actual = life_cycle_mediator_.getStatus(info);
    EXPECT_EQ(expected_status, actual) << life_cycle_mediator_.statusToString(expected_status) << " not equal to " 
    << life_cycle_mediator_.statusToString(actual);
  }
}

TEST(LifeCycleMediator, getAndSetStatus_ALL)
{
  vector<LifeCycleStatus> all = AMLifeCycleMediator::getLifeCycleStatuses();

  for (int i = 0; i < all.size(); i++) //exclude ERROR_PROCESSING
  {
    EXPECT_LIFE_CYCLE_STATUS(all[i], true);
  }
}

void EXPECT_LIFE_CYCLE_STATE(LifeCycleState expected_state,bool expected_success)
{
  AMLifeCycleMediator::LifeCycleInfo info;
  bool success = life_cycle_mediator_.setState(expected_state, info);
  EXPECT_EQ(expected_success, success);

  if(success)
  {
    LifeCycleState actual = life_cycle_mediator_.getState(info);
    EXPECT_EQ(expected_state, actual) << life_cycle_mediator_.stateToString(expected_state) << " not equal to " 
    << life_cycle_mediator_.stateToString(actual);
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
    str = life_cycle_mediator_.commandToString(allCommands[i]);
    EXPECT_TRUE(life_cycle_mediator_.stringToCommand(str, cmd));
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
    string_from_status = life_cycle_mediator_.statusToString(expected_status);
    bool success = life_cycle_mediator_.stringToStatus(string_from_status, status_from_string);
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
    str = life_cycle_mediator_.stateToString(allStates[i]);
    EXPECT_TRUE(life_cycle_mediator_.stringToState(str, state));
    EXPECT_EQ(allStates[i], state);
  }
}

TEST(LifeCycleMediator, statusToString_BadStatusReturnsEmptyString)
{
  LifeCycleStatus badStatus = LifeCycleStatus(-3);
  string str;

  str = life_cycle_mediator_.statusToString(badStatus);
  EXPECT_EQ(str, "");
}

TEST(LifeCycleMediator, stateToString_BadStateReturnsINVALID)
{
  LifeCycleState badState = LifeCycleState(-3);
  string str;

  str = life_cycle_mediator_.stateToString(badState);
  EXPECT_EQ(str, "INVALID");
}

TEST(LifeCycleMediator, commandToString_BadCommandReturnsEmptyString)
{
  LifeCycleCommand badCommand = LifeCycleCommand(-3);
  string str;

  str = life_cycle_mediator_.commandToString(badCommand);
  EXPECT_EQ(str, "");
}

TEST(LifeCycleMediator, stringToStatus_BadStringReturnsFalse)
{
  LifeCycleStatus initial_status = LifeCycleStatus::OK;
  LifeCycleStatus status = initial_status;
  string bad_string;

  bool bad_strings_should_return_false = life_cycle_mediator_.stringToStatus(bad_string, status);
  EXPECT_FALSE(bad_strings_should_return_false);
  EXPECT_EQ(status, initial_status) << "Status should be unchanged since the string is bad";
}

TEST(LifeCycleMediator, stringToCommand_BadStringReturnsFalse)
{
  LifeCycleCommand initial_cmd = LifeCycleCommand::ACTIVATE;
  LifeCycleCommand cmd = initial_cmd;

  string bad_string;
  bool bad_strings_should_return_false = life_cycle_mediator_.stringToCommand(bad_string, cmd);
  EXPECT_FALSE(bad_strings_should_return_false);
  EXPECT_EQ(cmd, initial_cmd) << "Command should be unchanged since the string is bad";
}

TEST(LifeCycleMediator, stringToState_BadStringReturnsFalse)
{
  LifeCycleState default_state=LifeCycleState::FINALIZED;
  LifeCycleState state = default_state;
  string bad_string;

  bool bad_strings_should_return_false = life_cycle_mediator_.stringToState(bad_string,state);
  EXPECT_FALSE(bad_strings_should_return_false);
  EXPECT_EQ(state,default_state) << "State should be unchanged since the string is bad";
}

TEST(LifeCycleMediator, setThrottleS_default)
{
  AMLifeCycleMediator::ThrottleInfo t;
  double setDefault = 0.0;

  AMLifeCycleMediator::ThrottleInfo default_t;

  life_cycle_mediator_.setThrottleS(setDefault, t);
  EXPECT_EQ(t.error_throttle_s, default_t.error_throttle_s);
  EXPECT_EQ(t.ok_throttle_s, default_t.ok_throttle_s);
  EXPECT_EQ(t.warn_throttle_s, default_t.warn_throttle_s);
}

TEST(LifeCycleMediator, setThrottleS_notDefault)
{
  AMLifeCycleMediator::ThrottleInfo t;
  double notDefault = 20;

  life_cycle_mediator_.setThrottleS(notDefault, t);
  EXPECT_EQ(t.error_throttle_s, notDefault);
  EXPECT_EQ(t.ok_throttle_s, notDefault);
  EXPECT_EQ(t.warn_throttle_s, notDefault);
}

TEST(LifeCycleMediator, getThrottle)
{
  double throttle;
  AMLifeCycleMediator::LifeCycleInfo info;
  AMLifeCycleMediator::ThrottleInfo default_t;

  info.status = LifeCycleStatus::OK;
  throttle = life_cycle_mediator_.getThrottle(info, default_t);
  EXPECT_EQ(throttle, default_t.ok_throttle_s);

  info.status = LifeCycleStatus::WARN;
  throttle = life_cycle_mediator_.getThrottle(info, default_t);
  EXPECT_EQ(throttle, default_t.warn_throttle_s);

  info.status = LifeCycleStatus::ERROR;
  throttle = life_cycle_mediator_.getThrottle(info, default_t);
  EXPECT_EQ(throttle, default_t.error_throttle_s);
}

bool EXPECT_EQ_SHUTDOWN(const LifeCycleState& state, bool expected)
{
  AMLifeCycleMediator::LifeCycleInfo info;
  info.state = state;
  EXPECT_EQ(life_cycle_mediator_.shutdown(info), expected);
}

TEST(LifeCycleMediator, shutdown)
{
  EXPECT_EQ_SHUTDOWN(LifeCycleState::UNCONFIGURED, true);
  EXPECT_EQ_SHUTDOWN(LifeCycleState::INACTIVE, true);
  EXPECT_EQ_SHUTDOWN(LifeCycleState::ACTIVE, true);

  EXPECT_EQ_SHUTDOWN(LifeCycleState::ACTIVATING, false);
  EXPECT_EQ_SHUTDOWN(LifeCycleState::CLEANING_UP, false);
  EXPECT_EQ_SHUTDOWN(LifeCycleState::CONFIGURING, false);
  EXPECT_EQ_SHUTDOWN(LifeCycleState::DEACTIVATING, false);
  EXPECT_EQ_SHUTDOWN(LifeCycleState::ERROR_PROCESSING, false);
  EXPECT_EQ_SHUTDOWN(LifeCycleState::FINALIZED, false);
  EXPECT_EQ_SHUTDOWN(LifeCycleState::INVALID, false);
  EXPECT_EQ_SHUTDOWN(LifeCycleState::SHUTTING_DOWN, false);
}


