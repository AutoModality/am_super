#include <gtest/gtest.h>  // googletest header file
#include <am_super/state_mediator.h>
#include <am_super/super_state.h>

#include <string>
using std::string;
using namespace am;

StateMediator mediator;

void
ASSERT_TRANSITION_ALLOWED(SuperState from, SuperState to, bool expected)
{
  bool allowed = mediator.allowsTransition(from, to);
  EXPECT_EQ(expected,allowed)  << "For state: " + std::to_string((int)to);
}

void
ASSERT_TRANSITIONS_ALLOWED(SuperState from, std::vector<SuperState> states, bool expected)
{
  for(int i = 0; i < states.size(); i++)
  {
    SuperState state = states.at(i);
    ASSERT_TRANSITION_ALLOWED(from,state,expected);
  }
}


TEST(StateMediator, allowsTransition_OnlyOffToBootingIAllowed)
{
  ASSERT_TRANSITION_ALLOWED(SuperState::OFF,SuperState::BOOTING,true);
  std::vector<SuperState> notAllowed = mediator.allSuperStates();
  notAllowed.erase(notAllowed.begin() + (int) SuperState::BOOTING)  ;
  ASSERT_TRANSITIONS_ALLOWED(SuperState::OFF,notAllowed,false);
}


TEST(StateMediator, allowsTransition_OnlyBootingToReadyIsAllowed)
{
  ASSERT_TRANSITION_ALLOWED(SuperState::BOOTING,SuperState::READY,true);
  std::vector<SuperState> notAllowed = mediator.allSuperStates();
  notAllowed.erase(notAllowed.begin() + (int) SuperState::READY);
  ASSERT_TRANSITIONS_ALLOWED(SuperState::BOOTING,notAllowed,false);
}


TEST(StateMediator, allSuperStates_IncludesAll)
{
  std::vector<SuperState> all = mediator.allSuperStates();
  int expectedNumberOfStates =11;
  int actualNumberOfStates = all.size();
  ASSERT_EQ(expectedNumberOfStates,actualNumberOfStates);
  ASSERT_EQ(all.at((int) SuperState::OFF),SuperState::OFF); 
  ASSERT_EQ(all.at((int) SuperState::ABORT),SuperState::ABORT); 
  ASSERT_EQ(all.at((int) SuperState::ARMED),SuperState::ARMED); 
  ASSERT_EQ(all.at((int) SuperState::ARMING),SuperState::ARMING); 
  ASSERT_EQ(all.at((int) SuperState::SEMI_AUTO),SuperState::SEMI_AUTO); 
  ASSERT_EQ(all.at((int) SuperState::SHUTDOWN),SuperState::SHUTDOWN); 
  ASSERT_EQ(all.front(),SuperState::OFF); 
  ASSERT_EQ(all.back(),SuperState::SHUTDOWN);
}
