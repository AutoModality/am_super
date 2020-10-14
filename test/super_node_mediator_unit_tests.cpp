#include <gtest/gtest.h>  // googletest header file
#include <am_super/super_node_mediator.h>
#include <boost/algorithm/string/join.hpp>

using namespace std;
using namespace am;

const string SUPER_NODE_NAME = "my_super_node";

SuperNodeMediator superNodeMediator(SUPER_NODE_NAME);

TEST(Node, nodeNameStripped_RemovesLeadingSlash)
{
  std::string name = "/something";
  std::string stripped = superNodeMediator.nodeNameStripped(name);
  ASSERT_EQ(stripped, "something") << "Leading slash should be removed";
  ASSERT_EQ(name, "/something") << "Original is not modified";
}

TEST(Node, nodeNameStripped_RegularNameNotModified)
{
  std::string name = "something";
  std::string stripped = superNodeMediator.nodeNameStripped(name);
  ASSERT_EQ(stripped, "something") << "Nothing should be removed";
}

TEST(Node, nodeNameStripped_EmptyStringDoesNotExplode)
{
  std::string name = "";
  std::string stripped = superNodeMediator.nodeNameStripped(name);
  ASSERT_EQ(stripped, "") << "Safety check for index of a string";
}

TEST(Node, nodeNameIsSuper_ShouldEqualAmSuperWithoutSlash)
{
  ASSERT_TRUE(superNodeMediator.nodeNameIsSuper(SUPER_NODE_NAME));
}

TEST(Node, nodeNameIsSuper_ShouldEqualAmSuperWithSlash)
{
  ASSERT_TRUE(superNodeMediator.nodeNameIsSuper("/" + SUPER_NODE_NAME));
}
TEST(Node, nodeNameIsSuper_ShouldNotEqualMissingA)
{
  ASSERT_FALSE(superNodeMediator.nodeNameIsSuper("m_super"));
}

TEST(Node, nodeNameStripped_SuperAddedToManifest)
{
  SuperNodeMediator::Supervisor supervisor;
  ASSERT_EQ(supervisor.manifest.size(),0);
  superNodeMediator.addSuperToManifest(supervisor);
  ASSERT_EQ(supervisor.manifest.size(),1);
  string node_name = supervisor.manifest.at(0);
  ASSERT_TRUE(superNodeMediator.nodeNameIsSuper(node_name));
  
}

TEST(Node, initializeManifestedNode_FieldsAreSetProperly)
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

void ASSERT_CHECK(std::function<bool(SuperNodeMediator::Supervisor&, SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check, LifeCycleState state, bool expected,
  OperatorCommand last_op_command_received = OperatorCommand::ARM)
{
  SuperNodeMediator::SuperNodeInfo info;
  SuperNodeMediator::Supervisor supervisor;
  info.state = state;
  supervisor.last_op_command_received = last_op_command_received;
  ASSERT_EQ(check(supervisor, info, superNodeMediator), expected) << "For state: " + std::to_string((int)state);
}

TEST(Node, checkReadyToArm_All)
{
  std::function<bool(SuperNodeMediator::Supervisor&, SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> function = SuperNodeMediator::checkReadyToArm;
  ASSERT_CHECK(function, LifeCycleState::INVALID, false);
  ASSERT_CHECK(function, LifeCycleState::UNCONFIGURED, false);
  ASSERT_CHECK(function, LifeCycleState::INACTIVE, true);
  ASSERT_CHECK(function, LifeCycleState::ACTIVE, false); //assuming the node isn't super
  ASSERT_CHECK(function, LifeCycleState::FINALIZED, false);
  ASSERT_CHECK(function, LifeCycleState::CONFIGURING, false);
  ASSERT_CHECK(function, LifeCycleState::CLEANING_UP, false);
  ASSERT_CHECK(function, LifeCycleState::SHUTTING_DOWN, false);
  ASSERT_CHECK(function, LifeCycleState::ACTIVATING, false);
  ASSERT_CHECK(function, LifeCycleState::DEACTIVATING, false);
}

TEST(Node, checkOperatorSignaledToArm)
{
  std::function<bool(SuperNodeMediator::Supervisor&, SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> function = SuperNodeMediator::checkOperatorSignaledToArm;

  ASSERT_CHECK(function, (LifeCycleState)NULL, false, OperatorCommand::LAUNCH);
  ASSERT_CHECK(function, (LifeCycleState)NULL, true, OperatorCommand::ARM);
}

TEST(Node, checkArmed_All)
{
  std::function<bool(SuperNodeMediator::Supervisor&, SuperNodeMediator::SuperNodeInfo&,SuperNodeMediator&)> function = SuperNodeMediator::checkArmed;
  ASSERT_CHECK(function, LifeCycleState::INVALID, false);
  ASSERT_CHECK(function, LifeCycleState::UNCONFIGURED, false);
  ASSERT_CHECK(function, LifeCycleState::INACTIVE, false);
  ASSERT_CHECK(function, LifeCycleState::ACTIVE, true);
  ASSERT_CHECK(function, LifeCycleState::FINALIZED, false);
  ASSERT_CHECK(function, LifeCycleState::CONFIGURING, false);
  ASSERT_CHECK(function, LifeCycleState::CLEANING_UP, false);
  ASSERT_CHECK(function, LifeCycleState::SHUTTING_DOWN, false);
  ASSERT_CHECK(function, LifeCycleState::ACTIVATING, false);
  ASSERT_CHECK(function, LifeCycleState::DEACTIVATING, false);
}

TEST(Node, checkOperatorSignaledToLaunch)
{
  std::function<bool(SuperNodeMediator::Supervisor&, SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> function = SuperNodeMediator::checkOperatorSignaledToLaunch;

  ASSERT_CHECK(function, (LifeCycleState)NULL, false, OperatorCommand::ARM);
  ASSERT_CHECK(function, (LifeCycleState)NULL, true, OperatorCommand::LAUNCH);
}

void assertAllManifestedNodesCheck(bool expected_success, SuperNodeMediator::SuperNodeInfo node, bool check_result,
                                   string expected_error_code = "")
{
  std::function<bool(SuperNodeMediator::Supervisor&, SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check = [check_result](SuperNodeMediator::Supervisor&, SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&) {
    return check_result;
  };
  SuperNodeMediator::Supervisor supervisor;
  string expected_node_name = node.name.empty()? "test-node": node.name;
  node.name = expected_node_name;
  supervisor.nodes.insert({ node.name, node });
  pair<bool, map<string, string>> result = superNodeMediator.allManifestedNodesCheck(supervisor, check);
  map<string, string> error_messages = result.second;
  bool success = result.first;
  ASSERT_EQ(success, expected_success) << error_messages.begin()->second;
  if (!expected_error_code.empty())
  {
    ASSERT_EQ(error_messages.size(),1);
    string node_name = error_messages.begin()->first;
    string error_message = error_messages.begin()->second;
    ASSERT_EQ(node_name, expected_node_name);
    ASSERT_TRUE(error_message.rfind(expected_error_code, 0) == 0) << error_message;
  }
}

TEST(Node, allManifestedNodesCheck_NonManifestIsSuccess)
{
  SuperNodeMediator::SuperNodeInfo node;
  node.manifested = false;
  bool check_result = false;
  assertAllManifestedNodesCheck(true, node, false);
}

TEST(Node, allManifestedNodesCheck_SuccessfulMnaifestedNode)
{
  SuperNodeMediator::SuperNodeInfo node;
  node.manifested = true;
  node.online = true;
  node.status = LifeCycleStatus::OK;
  bool check_result = true;

  assertAllManifestedNodesCheck(true, node, true);
}

TEST(Node, allManifestedNodesCheck_NotOnlineReturnsFalse)
{
  SuperNodeMediator::SuperNodeInfo node;
  node.manifested = true;
  node.online = false;
  assertAllManifestedNodesCheck(false, node, true, "[U5JB]");
}

TEST(Node, allManifestedNodesCheck_CheckReturnsFalseIsFailure)
{
  SuperNodeMediator::SuperNodeInfo node;
  node.state = LifeCycleState::CLEANING_UP;
  node.manifested = true;
  node.online = true;
  assertAllManifestedNodesCheck(false, node, false, "[2OQ0]");
}

TEST(Node, allManifestedNodesCheck_ErrorStatusReturnsFalse)
{
  SuperNodeMediator::SuperNodeInfo node;
  node.manifested = true;
  node.online = true;
  node.status = LifeCycleStatus::ERROR;
  assertAllManifestedNodesCheck(false, node, true, "[AA0A]");
}

[[deprecated]]
TEST(Node, allManifestedNodesCheck_FlightControllerLifeCycleNotYetImplementedSkipsCheck)
{
  SuperNodeMediator::SuperNodeInfo node;
  node.manifested = true;
  node.online = true;
  node.name = "flight_controller";
  bool expected_success, check_result;
  assertAllManifestedNodesCheck(expected_success = true, node, check_result = false, "[WCK2]");
}

TEST(Node, parseManifest_EmptyManifest)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "");
  ASSERT_EQ(supervisor.manifest.size(), 0);
}

TEST(Node, parseManifest_SpacesStripped)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "first, second, third");
  vector<string> expected{ "first", "second", "third" };
  ASSERT_EQ(supervisor.manifest.size(), 3);
  ASSERT_EQ(supervisor.manifest, expected);
}


TEST(Node, parseManifest_TrailingCommaIsIgnored)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "first,");
  vector<string> expected{ "first"};
  EXPECT_EQ(supervisor.manifest.size(), 1);
  EXPECT_EQ(supervisor.manifest, expected);
}

TEST(Node, parseManifest_EmptyCommaInMiddleIsIgnored)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "first,,second");
  vector<string> expected{ "first","second"};
  EXPECT_EQ(supervisor.manifest.size(), 2);
  EXPECT_EQ(supervisor.manifest, expected);
}

TEST(Node, parseManifest_Single)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "first");
  vector<string> expected{ "first" };
  ASSERT_EQ(supervisor.manifest.size(), 1);
  ASSERT_EQ(supervisor.manifest, expected);
}
TEST(Node, parseManifest_Double)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "first,second");
  vector<string> expected{ "first", "second" };
  ASSERT_EQ(supervisor.manifest.size(), 2);
  ASSERT_EQ(supervisor.manifest, expected);
}

TEST(Node, nodesOnlineCount_EmptyNodesIsZero)
{
  SuperNodeMediator::Supervisor supervisor;
  int count = superNodeMediator.nodesOnlineCount(supervisor);
  ASSERT_EQ(count, 0);
}

TEST(Node, nodesOnlineCount_OneNotOnlineIsZero)
{
  SuperNodeMediator::Supervisor supervisor;
  SuperNodeMediator::SuperNodeInfo node;
  node.online = false;
  supervisor.nodes.insert({ "whatever", node });
  int count = superNodeMediator.nodesOnlineCount(supervisor);
  ASSERT_EQ(count, 0);
}

TEST(Node, nodesOnlineCount_OneOnlineIsOne)
{
  SuperNodeMediator::Supervisor supervisor;
  SuperNodeMediator::SuperNodeInfo node;
  node.online = true;
  supervisor.nodes.insert({ "whatever", node });
  int count = superNodeMediator.nodesOnlineCount(supervisor);
  ASSERT_EQ(count, 1);
}

/**Combination of each of manifested and online testing both methods*/
TEST(Node, nodesOnlineCount_ManifestedOnlinedMixed)
{
  SuperNodeMediator::Supervisor supervisor;
  {
    SuperNodeMediator::SuperNodeInfo node;
    node.online = true;
    node.manifested = true;
    supervisor.nodes.insert({ "manifested-online", node });
  }
  {
    SuperNodeMediator::SuperNodeInfo node;
    node.online = false;
    node.manifested = true;
    supervisor.nodes.insert({ "manifested-not-online", node });
  }
  {
    SuperNodeMediator::SuperNodeInfo node;
    node.online = true;
    node.manifested = false;
    supervisor.nodes.insert({ "not-manifested-online", node });
  }
  {
    SuperNodeMediator::SuperNodeInfo node;
    node.online = false;
    node.manifested = false;
    supervisor.nodes.insert({ "not-manifested-not-online", node });
  }

  int online_count = superNodeMediator.nodesOnlineCount(supervisor);
  ASSERT_EQ(online_count, 2);
  int manifested_online_count = superNodeMediator.manifestedNodesOnlineCount(supervisor);
  ASSERT_EQ(manifested_online_count, 1) << "Only 1 is manifested and online";
}


TEST(Node, manifestedNodesNotOnline_RemovesNonManifestedNotOnline)
{
  SuperNodeMediator::Supervisor supervisor;
  {
    SuperNodeMediator::SuperNodeInfo manifestedOnlineToBeRemoved;
    manifestedOnlineToBeRemoved.online = true;
    manifestedOnlineToBeRemoved.manifested = true;
    supervisor.nodes.insert({ "manifestedOnlineToBeRemoved", manifestedOnlineToBeRemoved });
  }

  {
    SuperNodeMediator::SuperNodeInfo nonManifestedNotOnlineToBeRemoved;
    nonManifestedNotOnlineToBeRemoved.online = false;
    nonManifestedNotOnlineToBeRemoved.manifested = false;
    supervisor.nodes.insert({ "nonManifestedNotOnlineToBeRemoved", nonManifestedNotOnlineToBeRemoved });
  }

  {
    SuperNodeMediator::SuperNodeInfo manifestedNotOnlineToBeKept;
    manifestedNotOnlineToBeKept.online = false;
    manifestedNotOnlineToBeKept.manifested = true;
    supervisor.nodes.insert({ "manifestedNotOnlineToBeKept", manifestedNotOnlineToBeKept });
  }

  {
    //prove it works for 1
    map<string,SuperNodeMediator::SuperNodeInfo> results = superNodeMediator.manifestedNodesNotOnline(supervisor);
    string names=superNodeMediator.manifestedNodesNotOnlineNamesList(supervisor);
    EXPECT_EQ(results.size(), 1);
    EXPECT_EQ(results.count("manifestedNotOnlineToBeKept"),1);
    EXPECT_EQ(names,"manifestedNotOnlineToBeKept");
  }

  //prove it works for 2
  {
    SuperNodeMediator::SuperNodeInfo manifestedNotOnlineToBeKept;
    manifestedNotOnlineToBeKept.online = false;
    manifestedNotOnlineToBeKept.manifested = true;
    supervisor.nodes.insert({ "anotherKeeper", manifestedNotOnlineToBeKept });
  }

  {
    map<string,SuperNodeMediator::SuperNodeInfo> results = superNodeMediator.manifestedNodesNotOnline(supervisor);
    string names=superNodeMediator.manifestedNodesNotOnlineNamesList(supervisor);
    EXPECT_EQ(results.size(), 2);
    EXPECT_EQ(results.count("manifestedNotOnlineToBeKept"),1);
    EXPECT_EQ(results.count("anotherKeeper"),1);
    EXPECT_EQ(names,"anotherKeeper, manifestedNotOnlineToBeKept") << "order might be unpredictable if this shows up as flaky";
  }
}

TEST(Node, setControllerState)
{
  SuperNodeMediator::Supervisor supervisor;

  ControllerState state = ControllerState::COMPLETED;
  supervisor.session_completed = false;
  superNodeMediator.setControllerState(supervisor, state);

  ASSERT_TRUE(supervisor.session_completed);
}

TEST(Node, setOperatorCommand)
{
  SuperNodeMediator::Supervisor supervisor;
  OperatorCommand command;

  command = OperatorCommand::ABORT;
  superNodeMediator.setOperatorCommand(supervisor, command);
  ASSERT_EQ(supervisor.last_op_command_received, command);

  command = OperatorCommand::ARM;
  superNodeMediator.setOperatorCommand(supervisor, command);
  ASSERT_EQ(supervisor.last_op_command_received, command);
}

void ASSERT_getStateTransition(const SuperState &current_state, const SuperState &expected_state, const OperatorCommand &operator_command = (OperatorCommand)-1)
{
  SuperNodeMediator::Supervisor supervisor;
  supervisor.system_state = current_state;
  supervisor.last_op_command_received = operator_command;
  SuperNodeMediator::StateTransition t = superNodeMediator.getStateTransition(supervisor);
  ASSERT_EQ(t.to_state, expected_state) << "StateTransition to_state not equal to expected_state";
}

TEST(Node, getStateTransition_BootingToReady)
{
  ASSERT_getStateTransition(SuperState::BOOTING, SuperState::READY);
}

TEST(Node, getStateTransition_ReadyToArming)
{
  ASSERT_getStateTransition(SuperState::READY, SuperState::ARMING, OperatorCommand::ARM);
}

TEST(Node, getStateTransition_ArmingToArmed)
{
  ASSERT_getStateTransition(SuperState::ARMING, SuperState::ARMED);
}

TEST(Node, getStateTransition_ArmedToAutoWhenOperatorSendsLaunch)
{
  ASSERT_getStateTransition(SuperState::ARMED, SuperState::AUTO, OperatorCommand::LAUNCH);
}

TEST(Node, DISABLED_getStateTransition_AutoToDisarmingWhenControllerStateIsCompleted)
{
  
}

TEST(Node, getStateTransition_DisarmingToReady)
{
  ASSERT_getStateTransition(SuperState::DISARMING, SuperState::READY);
}

TEST(Node, DISABLED_getStateTransition_AutoToManualWhenOperatorSendsManual)
{
  ASSERT_getStateTransition(SuperState::AUTO, SuperState::MANUAL, OperatorCommand::MANUAL);
}