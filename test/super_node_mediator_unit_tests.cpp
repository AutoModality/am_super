#include <gtest/gtest.h>  // googletest header file
#include <am_super/super_node_mediator.h>

using namespace std;
using namespace am;

SuperNodeMediator superNodeMediator;

TEST(SuperNodeMediator, nodeNameStripped_RemovesLeadingSlash)
{
  std::string name = "/something";
  std::string stripped = superNodeMediator.nodeNameStripped(name);
  ASSERT_EQ(stripped, "something") << "Leading slash should be removed";
  ASSERT_EQ(name, "/something") << "Original is not modified";
}

TEST(SuperNodeMediator, nodeNameStripped_RegularNameNotModified)
{
  std::string name = "something";
  std::string stripped = superNodeMediator.nodeNameStripped(name);
  ASSERT_EQ(stripped, "something") << "Nothing should be removed";
}

TEST(SuperNodeMediator, nodeNameStripped_EmptyStringDoesNotExplode)
{
  std::string name = "";
  std::string stripped = superNodeMediator.nodeNameStripped(name);
  ASSERT_EQ(stripped, "") << "Safety check for index of a string";
}

TEST(SuperNodeMediator, initializeManifestedNode_FieldsAreSetProperly)
{
  std::string name = "node1";
  SuperNodeMediator::SuperNodeInfo nodeInfo = superNodeMediator.initializeManifestedNode(name);
  ASSERT_EQ(nodeInfo.name, name) << "Safety check for index of a string";
  ASSERT_EQ(nodeInfo.pid, -1) << "Doesn't really matter...references purposes only";
  ASSERT_EQ(nodeInfo.online, false) << "Initialized, but not yet reported anything";
  ASSERT_EQ(nodeInfo.manifested, true) << "Declared in the manifest";
  ASSERT_EQ(nodeInfo.state, LifeCycleState::UNCONFIGURED) << "Beginning of state lifecycle";
  ASSERT_EQ(nodeInfo.status, LifeCycleStatus::OK) << "All is good until reported otherwise";
}

void ASSERT_CHECK(std::function<bool(SuperNodeMediator::SuperNodeInfo&)> check, LifeCycleState state, bool expected)
{
  SuperNodeMediator::SuperNodeInfo info;
  info.state = state;
  ASSERT_EQ(check(info), expected) << "For state: " + std::to_string((int)state);
}


TEST(SuperNodeMediator,checkReadyForConfigureState_All)
{
  std::function<bool(SuperNodeMediator::SuperNodeInfo&)> function = SuperNodeMediator::checkReadyForConfigureState;
  ASSERT_CHECK(function,LifeCycleState::INVALID,false);
  ASSERT_CHECK(function,LifeCycleState::UNCONFIGURED,true);
  ASSERT_CHECK(function,LifeCycleState::INACTIVE,true);
  ASSERT_CHECK(function,LifeCycleState::ACTIVE,true);
  ASSERT_CHECK(function,LifeCycleState::FINALIZED,false);
  ASSERT_CHECK(function,LifeCycleState::CONFIGURING,false);
  ASSERT_CHECK(function,LifeCycleState::CLEANING_UP,false);
  ASSERT_CHECK(function,LifeCycleState::SHUTTING_DOWN,false);
  ASSERT_CHECK(function,LifeCycleState::ACTIVATING,false);
  ASSERT_CHECK(function,LifeCycleState::DEACTIVATING,false);
}

TEST(SuperNodeMediator,checkReadyForActivateState_All)
{
  std::function<bool(SuperNodeMediator::SuperNodeInfo&)> function = SuperNodeMediator::checkReadyForActivateState;
  ASSERT_CHECK(function,LifeCycleState::INVALID,false);
  ASSERT_CHECK(function,LifeCycleState::UNCONFIGURED,false);
  ASSERT_CHECK(function,LifeCycleState::INACTIVE,true);
  ASSERT_CHECK(function,LifeCycleState::ACTIVE,true);
  ASSERT_CHECK(function,LifeCycleState::FINALIZED,false);
  ASSERT_CHECK(function,LifeCycleState::CONFIGURING,false);
  ASSERT_CHECK(function,LifeCycleState::CLEANING_UP,false);
  ASSERT_CHECK(function,LifeCycleState::SHUTTING_DOWN,false);
  ASSERT_CHECK(function,LifeCycleState::ACTIVATING,false);
  ASSERT_CHECK(function,LifeCycleState::DEACTIVATING,false);
}
TEST(SuperNodeMediator,checkActivateState_All)
{
  std::function<bool(SuperNodeMediator::SuperNodeInfo&)> function = SuperNodeMediator::checkActivateState;
  ASSERT_CHECK(function,LifeCycleState::INVALID,false);
  ASSERT_CHECK(function,LifeCycleState::UNCONFIGURED,false);
  ASSERT_CHECK(function,LifeCycleState::INACTIVE,false);
  ASSERT_CHECK(function,LifeCycleState::ACTIVE,true);
  ASSERT_CHECK(function,LifeCycleState::FINALIZED,false);
  ASSERT_CHECK(function,LifeCycleState::CONFIGURING,false);
  ASSERT_CHECK(function,LifeCycleState::CLEANING_UP,false);
  ASSERT_CHECK(function,LifeCycleState::SHUTTING_DOWN,false);
  ASSERT_CHECK(function,LifeCycleState::ACTIVATING,false);
  ASSERT_CHECK(function,LifeCycleState::DEACTIVATING,false);
}

TEST(SuperNodeMediator,allManifestedNodesCheck_NonManifestIsSuccess)
{
  bool expected_success=true;
  string name = "not-manifested";
  SuperNodeMediator::Supervisor supervisor;
  SuperNodeMediator::SuperNodeInfo node;
  node.name="not manifested";
  node.manifested=false;
  supervisor.nodes.insert({node.name,node});
  pair<bool,map<string,string>> result = superNodeMediator.allManifestedNodesCheck(supervisor,NULL);
  ASSERT_EQ(get<0>(result),expected_success);
}


