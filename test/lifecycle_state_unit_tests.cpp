#include <gtest/gtest.h>  // googletest header file
#include <super_lib/am_life_cycle.h>

using namespace am;
using namespace std;



TEST(LifeCycle, stateTestStringConversion)
{
  vector<LifeCycleState> allStates = AMLifeCycle::allLifeCycleStates();
  string str;
  LifeCycleState state, state_final;
}

