#include <gtest/gtest.h>  // googletest header file
#include <am_super/state_mediator.h>
#include <am_super/super_state.h>

#include <string>
using std::string;
using namespace am;

StateMediator mediator;

/**Re-usable test method for validating single states*/
void ASSERT_TRANSITION_ALLOWED(SuperState from, SuperState to, bool expected)
{
  bool allowed = mediator.allowsTransition(from, to);
  EXPECT_EQ(expected, allowed) << "For state: " + std::to_string((int)to);
}

/**Re-usable test method for validating multiple states behave as expected*/
void ASSERT_TRANSITIONS_ALLOWED(SuperState from, std::vector<SuperState> states, bool expected)
{
  for (int i = 0; i < states.size(); i++)
  {
    SuperState state = states.at(i);
    ASSERT_TRANSITION_ALLOWED(from, state, expected);
  }
}

/**Re-usable method for validating the states given are allowed and all other states
 * are not allowed
 */
void ASSERT_MULTIPLE_STATES_ALLOWED(SuperState from, std::vector<SuperState> allowedStates)
{
  // verify allowed as expected
  for (int i = 0; i < allowedStates.size(); i++)
  {
    SuperState to = allowedStates.at(i);
    ASSERT_TRANSITION_ALLOWED(from, to, true);
  }

  // not allowed is all states minus those allowed
  std::vector<SuperState> notAllowed;
  for (SuperState state : mediator.allSuperStates())
  {
    if (std::find(allowedStates.begin(), allowedStates.end(), state) == allowedStates.end())
    {
      notAllowed.push_back(state);
    }
  }

  ASSERT_TRANSITIONS_ALLOWED(from, notAllowed, false);
}

/**Common situation to ensure a single state transition is allowed*/
void ASSERT_SINGLE_STATE_ALLOWED(SuperState from, SuperState to)
{
  std::vector<SuperState> allowed{ to };
  ASSERT_MULTIPLE_STATES_ALLOWED(from, allowed);
}

TEST(StateMediator, allowsTransition_OnlyOffToBootingIAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::OFF, SuperState::BOOTING);
}

TEST(StateMediator, allowsTransition_OnlyBootingToReadyIsAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::BOOTING, SuperState::READY);
}

TEST(StateMediator, allowsTransition_OnlyReadyToArmingIsAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::READY, SuperState::ARMING);
}

TEST(StateMediator, allowsTransition_OnlyArmingToArmedIsAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::ARMING, SuperState::ARMED);
}

TEST(StateMediator, allowsTransition_ArmedToAutoAndAbortIsAllowed)
{
  std::vector<SuperState> allowed{ SuperState::ABORT, SuperState::AUTO };
  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::ARMED, allowed);
}

TEST(StateMediator, allowsTransition_HoldToExceptionStates)
{
  std::vector<SuperState> allowed{ SuperState::ABORT, SuperState::MANUAL };

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::HOLD, allowed);
}

TEST(StateMediator, allowsTransition_AbortToReadyAndManualAllowed)
{
  std::vector<SuperState> allowed{ SuperState::READY, SuperState::MANUAL };

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::ABORT, allowed);
}

TEST(StateMediator, allowsTransition_OnlyManualToReadyIsAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::MANUAL, SuperState::READY);
}

TEST(StateMediator, allowsTransition_ShutdownToOffAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::SHUTDOWN, SuperState::OFF);
}

TEST(StateMediator, allowsTransition_InvalidStateHandled)
{
  
  int someBadNumber = 999999;
  ASSERT_ANY_THROW(mediator.allowsTransition((SuperState)someBadNumber, SuperState::OFF))
  << "Not registering a state NotAllowedNoThrowis a coding error OR passing an invalid state is too.";
  ASSERT_TRANSITION_ALLOWED(SuperState::OFF, (SuperState)someBadNumber, false);
}

TEST(StateMediator, allowsTransition_AutoToManyAllowed)
{
  std::vector<SuperState> allowed{ SuperState::HOLD, SuperState::ABORT, SuperState::READY, SuperState::SEMI_AUTO,
                                   SuperState::MANUAL };

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::AUTO, allowed);
}
TEST(StateMediator, allowsTransition_SemiAutoToManyAllowed)
{
  std::vector<SuperState> allowed{ SuperState::AUTO, SuperState::HOLD, SuperState::ABORT, SuperState::MANUAL };

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::SEMI_AUTO, allowed);
}

TEST(StateMediator, stateToString_AllStatesHaveString)
{
  for (SuperState state : mediator.allSuperStates())
  {
    std::string_view str = mediator.stateToString(state);
    ASSERT_NE(str, "");
  }
}

TEST(StateMediator, stateToString_InvalidStateReturnsInvalidString)
{
  ASSERT_ANY_THROW(mediator.stateToString((SuperState)999999));
}

/**Basic validation of super state enumeration */
TEST(StateMediator, allSuperStates_IncludesAll)
{
  std::vector<SuperState> all = mediator.allSuperStates();
  int expectedNumberOfStates = 11;
  int actualNumberOfStates = all.size();
  ASSERT_EQ(expectedNumberOfStates, actualNumberOfStates);
  ASSERT_EQ(all.at((int)SuperState::OFF), SuperState::OFF);
  ASSERT_EQ(all.at((int)SuperState::ABORT), SuperState::ABORT);
  ASSERT_EQ(all.at((int)SuperState::ARMED), SuperState::ARMED);
  ASSERT_EQ(all.at((int)SuperState::ARMING), SuperState::ARMING);
  ASSERT_EQ(all.at((int)SuperState::SEMI_AUTO), SuperState::SEMI_AUTO);
  ASSERT_EQ(all.at((int)SuperState::SHUTDOWN), SuperState::SHUTDOWN);
  ASSERT_EQ(all.front(), SuperState::OFF);
  ASSERT_EQ(all.back(), SuperState::SHUTDOWN);
  ASSERT_EQ(all.back(), SuperState::LAST_STATE);
}
