#include <gtest/gtest.h>  // googletest header file
#include <am_super/super_node_mediator.h>

using namespace std;
using namespace am;

/** Validates state transitions.
 * https://automodality.atlassian.net/wiki/spaces/AMROS/pages/929234949/AMROS+System+States
 *
 */
class TransitionReady : public ::testing::Test
{
protected:
  SuperNodeMediator superNodeMediator;

  TransitionReady() : superNodeMediator("am_super"){}

  SuperNodeMediator::SuperNodeInfo manifested_online_node_fixture()
  {
    SuperNodeMediator::SuperNodeInfo node;
    node.online = true;
    node.manifested = true;
    return node;
  }
  /**The main test method providing reuse for testing states. Overloaded methods are provided
   * for ease of use.
   *
   * Three main tests are happening:
   * 1. State transition if the check equals expected
   * 2. State transition if the flight controller state matches desired states.
   * 3. Lifecycle command provided for check failures that want to encourage re-issue
   */
  void ASSERT_TRANSITION_READY(SuperNodeMediator superNodeMediator, SuperState from, LifeCycleState node_state,
                              SuperNodeMediator::SuperFltCtrlState flt_ctrl_state, bool expected_ready,
                              SuperState expected_state, bool expected_resend_life_cycle_command,
                              LifeCycleCommand life_cycle_command, OperatorCommand last_op_command_received = OperatorCommand::ARM,
                              ControllerState last_controller_state_received = SuperNodeMediator::StateTransition::NO_CONTROLLER_STATE)
  {
    SuperNodeMediator::Supervisor supervisor;
    supervisor.system_state = from;
    supervisor.flt_ctrl_state = flt_ctrl_state;
    supervisor.last_op_command_received = last_op_command_received;
    supervisor.last_controller_state_received = last_controller_state_received;

    {
      SuperNodeMediator::SuperNodeInfo node = manifested_online_node_fixture();
      node.state = node_state;
      supervisor.nodes.insert({ "manresend_life_cycle_commandifested-node-name", node });
    }
    SuperNodeMediator::TransitionInstructions result = superNodeMediator.transitionReady(supervisor);
    ASSERT_EQ(result.ready_for_transition, expected_ready);
    if (result.ready_for_transition)
    {
      ASSERT_EQ(result.new_state, expected_state);
    }

    ASSERT_EQ(result.resend_life_cycle_command, expected_resend_life_cycle_command);
    if (result.resend_life_cycle_command)
    {
      ASSERT_EQ(result.life_cycle_command, life_cycle_command);
    }
  }

  /**When flight controller states are expected.*/
  void ASSERT_TRANSITION_READY(SuperNodeMediator superNodeMediator, SuperState from, LifeCycleState node_state,
                              SuperNodeMediator::SuperFltCtrlState flt_ctrl_state, bool expected_ready,
                              SuperState expected_state)
  {
    ASSERT_TRANSITION_READY(superNodeMediator, from, node_state, flt_ctrl_state, expected_ready, expected_state, false,
                            (LifeCycleCommand)NULL);
  }

  /**For the simplest cases without flight controller or lifecycle commands*/
  void ASSERT_TRANSITION_READY(SuperNodeMediator superNodeMediator, SuperState from, LifeCycleState node_state,
                              bool expected_ready, SuperState expected_state = SuperState::OFF)
  {
    ASSERT_TRANSITION_READY(superNodeMediator, from, node_state, (SuperNodeMediator::SuperFltCtrlState)NULL,
                            expected_ready, expected_state);
  }
};


TEST_F(TransitionReady, transitionReady_BootingToReadyWhenAllNodesInactive)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::BOOTING, LifeCycleState::INACTIVE, true,
                          SuperState::READY);
}

TEST_F(TransitionReady, transitionReady_BootingNoTransitionWhenAllNodesAreActive)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::BOOTING, LifeCycleState::ACTIVE,
                        (SuperNodeMediator::SuperFltCtrlState)NULL, false, (SuperState)NULL, true,
                        LifeCycleCommand::CONFIGURE);
}

TEST_F(TransitionReady, transitionReady_BootingNoTransitionWhenNodesNotInactiveOrActive)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::BOOTING, LifeCycleState::UNCONFIGURED, 
                        (SuperNodeMediator::SuperFltCtrlState)NULL, false, (SuperState)NULL, true,
                        LifeCycleCommand::CONFIGURE);
}

/**Not ready to transition from Ready so send another configure command */
TEST_F(TransitionReady, transitionReady_ReadyNoTransitionWhenNotArmed)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::READY, (LifeCycleState)NULL,
                          (SuperNodeMediator::SuperFltCtrlState)NULL, false, (SuperState)NULL, false,
                          LifeCycleCommand::CONFIGURE, OperatorCommand::LAUNCH);
}

TEST_F(TransitionReady, transitionReady_ReadyToArmingWhenArmed)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::READY, LifeCycleState::INACTIVE,
                          (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::ARMING, false,
                          LifeCycleCommand::CONFIGURE, OperatorCommand::ARM);
}

TEST_F(TransitionReady, transitionReady_ReadyToShutdownOnOpShutdown)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::READY, LifeCycleState::SHUTTING_DOWN,
                          (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::SHUTDOWN, false,
                          LifeCycleCommand::SHUTDOWN, OperatorCommand::SHUTDOWN);

  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::READY, LifeCycleState::FINALIZED,
                          (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::SHUTDOWN, false,
                          LifeCycleCommand::SHUTDOWN, OperatorCommand::SHUTDOWN);
}


TEST_F(TransitionReady, transitionReady_ArmingToArmedWhenNodesActive)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::ARMING, LifeCycleState::ACTIVE, true, SuperState::ARMED);
}

TEST_F(TransitionReady, transitionReady_ArmingNoTransitionWhenNodesNotActive)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::ARMING, LifeCycleState::INACTIVE,
                          (SuperNodeMediator::SuperFltCtrlState)NULL, false, (SuperState)NULL, true,
                          LifeCycleCommand::ACTIVATE);
}
/* Enable when we implement multimap for SuperState to StateTransition*/
TEST_F(TransitionReady, DISABLED_transitionReady_ArmedToAbortWhenDectivated)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::ARMED, LifeCycleState::INACTIVE, true, SuperState::ABORT);
}
/* Enable when we implement multimap for SuperState to StateTransition*/
TEST_F(TransitionReady, DISABLED_transitionReady_ArmedNoTransitionRemainingActivated)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::ARMED, LifeCycleState::ACTIVE, false);
}

TEST_F(TransitionReady, transitionReady_ArmedToAutoWhenReadyToLaunchAndOperatorIsLaunching)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::ARMED, LifeCycleState::ACTIVE, 
                          (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::AUTO, 
                          false, (LifeCycleCommand)NULL, OperatorCommand::LAUNCH);
}

TEST_F(TransitionReady, transitionReady_AutoToSemiAutoOnPause)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::AUTO, LifeCycleState::ACTIVE, 
                        (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::SEMI_AUTO, 
                          false, (LifeCycleCommand)NULL, OperatorCommand::PAUSE);
}


TEST_F(TransitionReady, transitionReady_SemiAutoToAutoOnResume)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::SEMI_AUTO, LifeCycleState::ACTIVE, 
                        (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::AUTO, 
                          false, (LifeCycleCommand)NULL, OperatorCommand::RESUME);
}

TEST_F(TransitionReady, TransitionReady_AutoToAbortOnOperatorAbort)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::AUTO, LifeCycleState::ACTIVE, 
                        (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::ABORT, 
                          false, (LifeCycleCommand)NULL, OperatorCommand::ABORT);

}

TEST_F(TransitionReady, TransitionReady_AbortToManualOnOpManual)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::ABORT, LifeCycleState::ACTIVE, 
                        (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::MANUAL, 
                          true, LifeCycleCommand::DEACTIVATE, OperatorCommand::MANUAL);
}
TEST_F(TransitionReady, TransitionReady_SemiAutoToManualOnOpManual)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::SEMI_AUTO, LifeCycleState::ACTIVE, 
                        (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::MANUAL, 
                          true, LifeCycleCommand::DEACTIVATE, OperatorCommand::MANUAL);
}

TEST_F(TransitionReady, TransitionReady_AutoToManualOnOpManual)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::AUTO, LifeCycleState::ACTIVE, 
                        (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::MANUAL, 
                          true, LifeCycleCommand::DEACTIVATE, OperatorCommand::MANUAL);
}

TEST_F(TransitionReady, TransitionReady_ManualToDisarmingOnOpLanded)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::MANUAL, LifeCycleState::ACTIVE, 
                        (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::DISARMING, 
                          false, (LifeCycleCommand)NULL, SuperNodeMediator::StateTransition::NO_OPERATOR_COMMAND, 
                          ControllerState::COMPLETED);
}

TEST_F(TransitionReady, TransitionReady_AbortToDisarmingOnControllerStateCompleted)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::ABORT, LifeCycleState::ACTIVE, 
                        (SuperNodeMediator::SuperFltCtrlState)NULL, true, SuperState::DISARMING, 
                          false, (LifeCycleCommand)NULL, SuperNodeMediator::StateTransition::NO_OPERATOR_COMMAND, 
                          ControllerState::COMPLETED);
}

TEST_F(TransitionReady, DISABLED_transitionReady_AutoToAbortWhenDeactivated)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::AUTO, LifeCycleState::INACTIVE, true, SuperState::ABORT);
}

TEST_F(TransitionReady, DISABLED_transitionReady_ArmedNoTransitionWhenRemainingActive)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::AUTO, LifeCycleState::ACTIVE, false);
}

//======================= FLIGHT Controller States =====================================

TEST_F(TransitionReady, DISABLED_transitionReady_ArmedToAutoWhenFlightControllerIsAuto)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::ARMED, LifeCycleState::ACTIVE,
                          SuperNodeMediator::SuperFltCtrlState::AUTO, true, SuperState::AUTO);
}

TEST_F(TransitionReady, DISABLED_transitionReady_AutoToSemiAutoWhenFlightControllerIsHold)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::AUTO, LifeCycleState::ACTIVE,
                          SuperNodeMediator::SuperFltCtrlState::HOLD, true, SuperState::SEMI_AUTO);
}

TEST_F(TransitionReady, DISABLED_transitionReady_SemiAutoToAutoWhenFlightControllerIsHold)
{
  ASSERT_TRANSITION_READY(superNodeMediator, SuperState::SEMI_AUTO, LifeCycleState::ACTIVE,
                          SuperNodeMediator::SuperFltCtrlState::AUTO, true, SuperState::AUTO);
}
