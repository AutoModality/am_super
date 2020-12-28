#include <gtest/gtest.h>  // googletest header file
#include <am_super/super_state_mediator.h>
#include <am_super/super_state.h>

#include <string>
using std::string;
using namespace am;

class SuperStateMediatorTest : public ::testing::Test
{
protected:
  SuperStateMediator mediator;

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
};

TEST_F(SuperStateMediatorTest, allowsTransition_OnlyOffToBootingIsAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::OFF, SuperState::BOOTING);
}

TEST_F(SuperStateMediatorTest, allowsTransition_BootingToReadyAndShutdownAllowed)
{
  std::vector<SuperState> allowed{ SuperState::READY, SuperState::SHUTDOWN };
  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::BOOTING, allowed);
}

TEST_F(SuperStateMediatorTest, allowsTransition_ReadyToArmingAndShutdownAllowed)
{
  std::vector<SuperState> allowed{ SuperState::ARMING, SuperState::SHUTDOWN };
  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::READY, allowed);
}

TEST_F(SuperStateMediatorTest, allowsTransition_ArmingToArmedAndReadyAllowed)
{
  std::vector<SuperState> allowed{ SuperState::ARMED, SuperState::READY };
  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::ARMING, allowed);
}

TEST_F(SuperStateMediatorTest, allowsTransition_ArmedToAutoAndDisarmingAllowed)
{
  std::vector<SuperState> allowed{ SuperState::DISARMING, SuperState::AUTO };
  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::ARMED, allowed);
}

TEST_F(SuperStateMediatorTest, allowsTransition_HoldToExceptionStates)
{
  std::vector<SuperState> allowed{ SuperState::ABORT, SuperState::MANUAL };

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::HOLD, allowed);
}

TEST_F(SuperStateMediatorTest, allowsTransition_AbortToDisarmingAndManualAllowed)
{
  std::vector<SuperState> allowed{ SuperState::DISARMING, SuperState::MANUAL };

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::ABORT, allowed);
}

TEST_F(SuperStateMediatorTest, allowsTransition_OnlyManualToDisarmingIsAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::MANUAL, SuperState::DISARMING);
}

TEST_F(SuperStateMediatorTest, allowsTransition_ShutdownToOffAllowed)
{
  ASSERT_SINGLE_STATE_ALLOWED(SuperState::SHUTDOWN, SuperState::OFF);
}

TEST_F(SuperStateMediatorTest, allowsTransition_InvalidStateHandled)
{
  int someBadNumber = 999999;
  ASSERT_ANY_THROW(mediator.allowsTransition((SuperState)someBadNumber, SuperState::OFF))
      << "Not registering a state NotAllowedNoThrowis a coding error OR passing an invalid state is too.";
  ASSERT_TRANSITION_ALLOWED(SuperState::OFF, (SuperState)someBadNumber, false);
}

TEST_F(SuperStateMediatorTest, allowsTransition_AutoToManyAllowed)
{
  std::vector<SuperState> allowed{ SuperState::HOLD, SuperState::ABORT, SuperState::DISARMING, SuperState::SEMI_AUTO,
                                   SuperState::MANUAL };

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::AUTO, allowed);
}
TEST_F(SuperStateMediatorTest, allowsTransition_SemiAutoToManyAllowed)
{
  std::vector<SuperState> allowed{ SuperState::AUTO, SuperState::HOLD, SuperState::ABORT, SuperState::MANUAL };

  ASSERT_MULTIPLE_STATES_ALLOWED(SuperState::SEMI_AUTO, allowed);
}

TEST_F(SuperStateMediatorTest, stateToString_AllStatesHaveString)
{
  for (SuperState state : mediator.allSuperStates())
  {
    std::string_view str = mediator.stateToString(state);
    ASSERT_NE(str, "");
  }
}

TEST_F(SuperStateMediatorTest, stateToString_InvalidStateReturnsInvalidString)
{
  ASSERT_ANY_THROW(mediator.stateToString((SuperState)999999));
}

/**Basic validation of super state enumeration */
TEST_F(SuperStateMediatorTest, allSuperStates_IncludesAll)
{
  std::vector<SuperState> all = mediator.allSuperStates();
  int expectedNumberOfStates = 12;
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
}
