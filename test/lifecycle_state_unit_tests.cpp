#include <gtest/gtest.h>  // googletest header file
#include <super_lib/am_life_cycle.h>

using namespace am;
using namespace std;

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

