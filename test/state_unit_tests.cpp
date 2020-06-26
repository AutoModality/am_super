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



void
ASSERT_MULTIPLE_STATES_ALLOWED(SuperState from, std::vector<SuperState> allowedStates)
{
  std::vector<SuperState> notAllowed = mediator.allSuperStates();
  for(int i = 0; i < allowedStates.size(); i++)
  {
    SuperState to = allowedStates.at(i);
    ASSERT_TRANSITION_ALLOWED(from,to,true);
    notAllowed.erase(notAllowed.begin() + (int) to);
  }
  ASSERT_TRANSITIONS_ALLOWED(from,notAllowed,false);
}

/**Common situation to ensure a single state transition is allowed*/
void
ASSERT_SINGLE_STATE_ALLOWED(SuperState from, SuperState to)
{
  std::vector allowed{to};
  ASSERT_MULTIPLE_STATES_ALLOWED(from,allowed);
}

TEST(StateMediator, allowsTransition_OnlyOffToBootingIAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::OFF,SuperState::BOOTING);
}

TEST(StateMediator, allowsTransition_OnlyBootingToReadyIsAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::BOOTING,SuperState::READY);
}

TEST(StateMediator, allowsTransition_OnlyReadyToArmingIsAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::READY,SuperState::ARMING);
}

TEST(StateMediator, allowsTransition_OnlyArmingToArmedIsAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::ARMING,SuperState::ARMED);
}

TEST(StateMediator, allowsTransition_ArmedToAutoAndAbortIsAllowed)
{
  std::vector<SuperState> allowed{SuperState::ABORT, SuperState::AUTO};
  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::ARMED,allowed);
}




TEST(StateMediator, allowsTransition_HoldToExceptionStates)
{
  std::vector<SuperState> allowed{SuperState::ABORT, SuperState::MANUAL};

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::HOLD,allowed);
}

TEST(StateMediator, allowsTransition_AbortToReadyAndManualAllowed)
{
  std::vector<SuperState> allowed{SuperState::READY,SuperState::MANUAL};

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::ABORT,allowed);
}

TEST(StateMediator, allowsTransition_OnlyManualToReadyIsAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::MANUAL,SuperState::READY);
}

TEST(StateMediator, allowsTransition_ShutdownToOffAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::SHUTDOWN,SuperState::OFF);
}

TEST(StateMediator, allowsTransition_UnhandledThrowsException)
{
  FAIL() << "JUst testing";
}


TEST(StateMediator, allowsTransition_AutoToManyAllowed)
{
  std::vector<SuperState> allowed{SuperState::READY, SuperState::SEMI_AUTO, 
                                  SuperState::HOLD, SuperState::ABORT, 
                                  SuperState::MANUAL};

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::AUTO,allowed);
}
TEST(StateMediator, allowsTransition_SemiAutoToManyAllowed)
{
  std::vector<SuperState> allowed{SuperState::AUTO, SuperState::HOLD, SuperState::ABORT, 
                                  SuperState::MANUAL};

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::SEMI_AUTO,allowed);
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
  ASSERT_EQ(all.back(),SuperState::LAST_STATE);

}
