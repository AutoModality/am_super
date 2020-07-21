

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

void ASSERT_TRANSITION_READY( SuperNodeMediator superNodeMediator, SuperState from,LifeCycleState node_state,bool expected_ready,SuperState expected_state = SuperState::OFF)
{
SuperNodeMediator::Supervisor supervisor;
supervisor.system_state=from;
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


TEST_F(TransitionReady, transitionReady_BootingToReadyWithNoManifestedNodes)
{
bool expected_ready = true;
SuperState expected_state = SuperState::READY;
SuperNodeMediator::Supervisor supervisor;
supervisor.system_state=SuperState::BOOTING;
pair<bool,SuperState> result = superNodeMediator -> transitionReady(supervisor);
ASSERT_EQ(result.first,expected_ready);
ASSERT_EQ(result.second,expected_state);
}
