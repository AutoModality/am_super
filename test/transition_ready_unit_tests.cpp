

#include <gtest/gtest.h>  // googletest header file
#include <am_super/super_node_mediator.h>

using namespace std;
using namespace am;

class TransitionReady : public ::testing::Test
{
protected:
    SuperNodeMediator * superNodeMediator;

    SuperNodeMediator::SuperNodeInfo manifested_online_node_fixture()
    {
        SuperNodeMediator::SuperNodeInfo node;
        node.online=true;
        node.manifested=true;
        return node;
    }

 
};


SuperNodeMediator::SuperNodeInfo manifested_online_node_fixture()
{
    SuperNodeMediator::SuperNodeInfo node;
    node.online=true;
    node.manifested=true;
    return node;
}



void ASSERT_TRANSITION_READY( SuperNodeMediator superNodeMediator, SuperState from, LifeCycleState node_state,
                                 SuperNodeMediator::SuperFltCtrlState flt_ctrl_state, bool expected_ready, 
                                 SuperState expected_state)
{
    SuperNodeMediator::Supervisor supervisor;
    supervisor.system_state=from;
    supervisor.flt_ctrl_state=flt_ctrl_state;
    {
        SuperNodeMediator::SuperNodeInfo node = manifested_online_node_fixture();
        node.state=node_state;
        supervisor.nodes.insert({"manifested-node-name",node});
    }
    pair<bool,SuperState> result = superNodeMediator.transitionReady(supervisor);
    ASSERT_EQ(result.first,expected_ready);
    if(result.first)
    {
        ASSERT_EQ(result.second,expected_state);
    }
}

void ASSERT_TRANSITION_READY( SuperNodeMediator superNodeMediator, SuperState from,LifeCycleState node_state,
                                bool expected_ready,SuperState expected_state = SuperState::OFF)
{
    ASSERT_TRANSITION_READY(superNodeMediator,from,node_state,(SuperNodeMediator::SuperFltCtrlState) -1,expected_ready,expected_state);
}

TEST_F(TransitionReady, transitionReady_BootingToReadyWhenReadyToConfigure)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::BOOTING,LifeCycleState::UNCONFIGURED,true,SuperState::READY);
}

TEST_F(TransitionReady, transitionReady_BootingNoTransitionWhenNotReadyToConfigure)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::BOOTING,LifeCycleState::INVALID,false);
}

TEST_F(TransitionReady, transitionReady_ReadyNoTransitionWhenNotReadyToActivate)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::READY,LifeCycleState::INVALID,false);
}

TEST_F(TransitionReady, transitionReady_ReadyToArmingWhenReadyToActivate)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::READY,LifeCycleState::ACTIVE,true,SuperState::ARMING);
}

TEST_F(TransitionReady, transitionReady_ArmingToArmedWhenReadyToActivate)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::ARMING,LifeCycleState::ACTIVE,true,SuperState::ARMED);
}

TEST_F(TransitionReady, transitionReady_ArmingNoTransitionWhenNotReadyToActivate)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::ARMING,LifeCycleState::INVALID,false);
}

TEST_F(TransitionReady, transitionReady_ArmedToAbortWhenDectivated)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::ARMED,LifeCycleState::INACTIVE,true,SuperState::ABORT);
}

TEST_F(TransitionReady, transitionReady_ArmedNoTransitionRemainingActivated)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::ARMED,LifeCycleState::ACTIVE,false);
}

TEST_F(TransitionReady, transitionReady_AutoToAbortWhenDeactivated)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::AUTO,LifeCycleState::INACTIVE,true,SuperState::ABORT);
}

TEST_F(TransitionReady, transitionReady_ArmedNoTransitionWhenRemainingActive)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::AUTO,LifeCycleState::ACTIVE,false);
}

//======================= FLIGHT Controller States =====================================


TEST_F(TransitionReady, transitionReady_ArmedToAutoWhenFlightControllerIsAuto)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::ARMED,LifeCycleState::ACTIVE,
        SuperNodeMediator::SuperFltCtrlState::AUTO,true,SuperState::AUTO);
}

TEST_F(TransitionReady, transitionReady_ArmedToSemiAutoWhenFlightControllerIsHold)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::ARMED,LifeCycleState::ACTIVE,
        SuperNodeMediator::SuperFltCtrlState::HOLD,true,SuperState::SEMI_AUTO);
}


TEST_F(TransitionReady, transitionReady_AutoToSemiAutoWhenFlightControllerIsHold)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::AUTO,LifeCycleState::ACTIVE,
        SuperNodeMediator::SuperFltCtrlState::HOLD,true,SuperState::SEMI_AUTO);
}

TEST_F(TransitionReady, transitionReady_SemiAutoToAutoWhenFlightControllerIsHold)
{
    ASSERT_TRANSITION_READY(*superNodeMediator,SuperState::SEMI_AUTO,LifeCycleState::ACTIVE,
        SuperNodeMediator::SuperFltCtrlState::AUTO,true,SuperState::AUTO);
}