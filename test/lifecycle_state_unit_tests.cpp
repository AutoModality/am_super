#include <gtest/gtest.h>  // googletest header file
#include <super_lib/am_life_cycle.h>

using namespace am;
using namespace std;

TEST(LifeCycle, stateToString_BadStateReturnsInvalidString)
{
  LifeCycleState state = LifeCycleState(20);
  string str;
  str = AMLifeCycle::stateToString(state);
  ASSERT_EQ(str, "INVALID");
}

TEST(LifeCycle, stateTestStringConversion)
{
  vector<LifeCycleState> allStates = AMLifeCycle::allLifeCycleStates();
  string str;
  LifeCycleState state;

  for(int i = 0; i < allStates.size(); i++) 
  {
    str = AMLifeCycle::stateToString(allStates[i]);
    ASSERT_TRUE(AMLifeCycle::stringToState(str, state));
    ASSERT_EQ(allStates[i], state);
  }
}


TEST(LifeCycle, stateToString_BadStringReturnsFalse)
{
  LifeCycleState default_state=LifeCycleState::FINALIZED;
  LifeCycleState state = default_state;
  string bad_string;
  bool bad_strings_should_return_false = AMLifeCycle::stringToState(bad_string,state);
  ASSERT_FALSE(bad_strings_should_return_false);
  ASSERT_EQ(state,default_state) << "State should be unchanged since the string is bad";
}