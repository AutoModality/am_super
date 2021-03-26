#include <gtest/gtest.h>  // googletest header file
#include <am_super/super_node_mediator.h>
#include <boost/algorithm/string/join.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/join.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using namespace am;

class SuperNodeMediatorTest : public ::testing::Test
{
protected:
  const string SUPER_NODE_NAME = "my_super_node";
  SuperNodeMediator superNodeMediator;



  SuperNodeMediatorTest() : superNodeMediator(SUPER_NODE_NAME) {}

  SuperNodeMediator::SuperNodeInfo manifested_lifecycle_node(string node_name, LifeCycleState state)
  {
    SuperNodeMediator::SuperNodeInfo node;
    node.name = node_name;
    node.online = true;
    node.manifested = true;
    node.state = state;
    return node;
  }

  void ASSERT_CHECK(std::function<bool(SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check, LifeCycleState state, bool expected,
    OperatorCommand last_op_command_received = OperatorCommand::ARM, ControllerState last_controller_state_received = ControllerState::COMPLETED)
  {
    SuperNodeMediator::SuperNodeInfo info;
    SuperNodeMediator::Supervisor supervisor;
    info.state = state;
    supervisor.last_op_command_received = last_op_command_received;
    supervisor.last_controller_state_received = last_controller_state_received;
    ASSERT_EQ(check(info, superNodeMediator), expected) << "For state: " + std::to_string((int)state);
  }

  void assertAllManifestedNodesCheck(bool expected_success, SuperNodeMediator::SuperNodeInfo node, bool check_result,
                                   string expected_error_code = "")
  {
    std::function<bool(SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check = [check_result](SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&) {
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

  void assertAllManifestedNodesCheck(SuperNodeMediator::Supervisor supervisor,
                                    std::function<bool(SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> check, 
                                    vector<string> expected_failed_nodes, bool expected_success)
  {
    pair<bool, map<string, string>> result = superNodeMediator.allManifestedNodesCheck(supervisor, check);
    vector<string> failed_nodes;
    boost::copy(result.second | boost::adaptors::map_keys, std::back_inserter(failed_nodes));

    std::sort(failed_nodes.begin(), failed_nodes.end());
    std::sort(expected_failed_nodes.begin(), expected_failed_nodes.end());

    ASSERT_EQ((failed_nodes == expected_failed_nodes), expected_success);
  }

  void ASSERT_getStateTransition(const SuperState &current_state, const SuperState &expected_state, const OperatorCommand &operator_command = (OperatorCommand)-1, 
    const ControllerState& controller_state = (ControllerState)-1)
  {
    SuperNodeMediator::Supervisor supervisor;
    supervisor.system_state = current_state;
    supervisor.last_op_command_received = operator_command;
    supervisor.last_controller_state_received = controller_state;
    SuperNodeMediator::StateTransition t = superNodeMediator.getStateTransition(supervisor);
    ASSERT_EQ(t.to_state, expected_state) << "StateTransition to_state not equal to expected_state";
  }
};

TEST_F(SuperNodeMediatorTest, nodeNameStripped_RemovesLeadingSlash)
{
  std::string name = "/something";
  std::string stripped = superNodeMediator.nodeNameStripped(name);
  ASSERT_EQ(stripped, "something") << "Leading slash should be removed";
  ASSERT_EQ(name, "/something") << "Original is not modified";
}

TEST_F(SuperNodeMediatorTest, nodeNameStripped_RegularNameNotModified)
{
  std::string name = "something";
  std::string stripped = superNodeMediator.nodeNameStripped(name);
  ASSERT_EQ(stripped, "something") << "Nothing should be removed";
}

TEST_F(SuperNodeMediatorTest, nodeNameStripped_EmptyStringDoesNotExplode)
{
  std::string name = "";
  std::string stripped = superNodeMediator.nodeNameStripped(name);
  ASSERT_EQ(stripped, "") << "Safety check for index of a string";
}

TEST_F(SuperNodeMediatorTest, nodeNameIsSuper_ShouldEqualAmSuperWithoutSlash)
{
  ASSERT_TRUE(superNodeMediator.nodeNameIsSuper(SUPER_NODE_NAME));
}

TEST_F(SuperNodeMediatorTest, nodeNameIsSuper_ShouldEqualAmSuperWithSlash)
{
  ASSERT_TRUE(superNodeMediator.nodeNameIsSuper("/" + SUPER_NODE_NAME));
}
TEST_F(SuperNodeMediatorTest, nodeNameIsSuper_ShouldNotEqualMissingA)
{
  ASSERT_FALSE(superNodeMediator.nodeNameIsSuper("m_super"));
}

TEST_F(SuperNodeMediatorTest, nodeNameStripped_SuperAddedToManifest)
{
  SuperNodeMediator::Supervisor supervisor;
  ASSERT_EQ(supervisor.manifest.size(),0);
  superNodeMediator.addSuperToManifest(supervisor);
  ASSERT_EQ(supervisor.manifest.size(),1);
  string node_name = supervisor.manifest.at(0);
  ASSERT_TRUE(superNodeMediator.nodeNameIsSuper(node_name));
  
}

TEST_F(SuperNodeMediatorTest, initializeManifestedNode_FieldsAreSetProperly)
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

TEST_F(SuperNodeMediatorTest, checkReadyToArm_All)
{
  std::function<bool(SuperNodeMediator::SuperNodeInfo&, SuperNodeMediator&)> function = SuperNodeMediator::checkReadyToArm;
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

TEST_F(SuperNodeMediatorTest, checkArmed_All)
{
  std::function<bool(SuperNodeMediator::SuperNodeInfo&,SuperNodeMediator&)> function = SuperNodeMediator::checkArmed;
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

TEST_F(SuperNodeMediatorTest, allManifestedNodesCheck_NonManifestIsSuccess)
{
  SuperNodeMediator::SuperNodeInfo node;
  node.manifested = false;
  bool check_result = false;
  assertAllManifestedNodesCheck(true, node, false);
}

TEST_F(SuperNodeMediatorTest, allManifestedNodesCheck_SuccessfulMnaifestedNode)
{
  SuperNodeMediator::SuperNodeInfo node;
  node.manifested = true;
  node.online = true;
  node.status = LifeCycleStatus::OK;
  bool check_result = true;

  assertAllManifestedNodesCheck(true, node, true);
}

TEST_F(SuperNodeMediatorTest, allManifestedNodesCheck_NotOnlineReturnsFalse)
{
  SuperNodeMediator::SuperNodeInfo node;
  node.manifested = true;
  node.online = false;
  assertAllManifestedNodesCheck(false, node, true, "[U5JB]");
}

TEST_F(SuperNodeMediatorTest, allManifestedNodesCheck_CheckReturnsFalseIsFailure)
{
  SuperNodeMediator::SuperNodeInfo node;
  node.state = LifeCycleState::CLEANING_UP;
  node.manifested = true;
  node.online = true;
  assertAllManifestedNodesCheck(false, node, false, "[2OQ0]");
}

TEST_F(SuperNodeMediatorTest, allManifestedNodesCheck_ErrorStatusReturnsTrueButHasErrorMessage)
{
  SuperNodeMediator::SuperNodeInfo node;
  node.manifested = true;
  node.online = true;
  node.status = LifeCycleStatus::ERROR;
  assertAllManifestedNodesCheck(true, node, true, "[AA0A]");
}


TEST_F(SuperNodeMediatorTest, allManifestedNodesCheck_MultipleNodes_FirstNodeFails)
{
  SuperNodeMediator::Supervisor supervisor;
  supervisor.system_state = SuperState::BOOTING;

  //supervisor stores a map<string, SuperNodeInfo>
  supervisor.nodes.insert({"a", manifested_lifecycle_node("a", LifeCycleState::CONFIGURING)});
  supervisor.nodes.insert({"b", manifested_lifecycle_node("b", LifeCycleState::INACTIVE)});
  supervisor.nodes.insert({"c", manifested_lifecycle_node("c", LifeCycleState::INACTIVE)});
  supervisor.nodes.insert({"d", manifested_lifecycle_node("d", LifeCycleState::INACTIVE)});

  //a and c should fail
  vector<string> expected_failed_nodes({"a"});

  assertAllManifestedNodesCheck(supervisor, SuperNodeMediator::checkReadyToArm, expected_failed_nodes, true);
}

TEST_F(SuperNodeMediatorTest, allManifestedNodesCheck_MultipleNodes_TwoNodesFail)
{
  SuperNodeMediator::Supervisor supervisor;
  supervisor.system_state = SuperState::BOOTING;

  //supervisor stores a map<string, SuperNodeInfo>
  supervisor.nodes.insert({"a", manifested_lifecycle_node("a", LifeCycleState::CONFIGURING)});
  supervisor.nodes.insert({"b", manifested_lifecycle_node("b", LifeCycleState::CONFIGURING)});
  supervisor.nodes.insert({"c", manifested_lifecycle_node("c", LifeCycleState::INACTIVE)});
  supervisor.nodes.insert({"d", manifested_lifecycle_node("d", LifeCycleState::INACTIVE)});

  //a and b should fail
  vector<string> expected_failed_nodes({"a", "b"});

  assertAllManifestedNodesCheck(supervisor, SuperNodeMediator::checkReadyToArm, expected_failed_nodes, true);
}

TEST_F(SuperNodeMediatorTest, allManifestedNodesCheck_MultipleNodes_AllNodesFail)
{
  SuperNodeMediator::Supervisor supervisor;
  supervisor.system_state = SuperState::BOOTING;

  //supervisor stores a map<string, SuperNodeInfo>
  supervisor.nodes.insert({"a", manifested_lifecycle_node("a", LifeCycleState::CONFIGURING)});
  supervisor.nodes.insert({"b", manifested_lifecycle_node("b", LifeCycleState::CONFIGURING)});
  supervisor.nodes.insert({"c", manifested_lifecycle_node("c", LifeCycleState::CONFIGURING)});
  supervisor.nodes.insert({"d", manifested_lifecycle_node("d", LifeCycleState::CONFIGURING)});

  //all should fail
  vector<string> expected_failed_nodes({"a", "b", "c", "d"});

  assertAllManifestedNodesCheck(supervisor, SuperNodeMediator::checkReadyToArm, expected_failed_nodes, true);
}

TEST_F(SuperNodeMediatorTest, allManifestedNodesCheck_MultipleNodes_AllNodesPass)
{
  SuperNodeMediator::Supervisor supervisor;
  supervisor.system_state = SuperState::BOOTING;

  //supervisor stores a map<string, SuperNodeInfo>
  supervisor.nodes.insert({"a", manifested_lifecycle_node("a", LifeCycleState::INACTIVE)});
  supervisor.nodes.insert({"b", manifested_lifecycle_node("b", LifeCycleState::INACTIVE)});
  supervisor.nodes.insert({"c", manifested_lifecycle_node("c", LifeCycleState::INACTIVE)});
  supervisor.nodes.insert({"d", manifested_lifecycle_node("d", LifeCycleState::INACTIVE)});

  //none should fail
  vector<string> expected_failed_nodes({});

  assertAllManifestedNodesCheck(supervisor, SuperNodeMediator::checkReadyToArm, expected_failed_nodes, true);
}

TEST_F(SuperNodeMediatorTest, parseManifest_EmptyManifest)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "");
  ASSERT_EQ(supervisor.manifest.size(), 0);
}

TEST_F(SuperNodeMediatorTest, parseManifest_SpacesStripped)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "first, second, third");
  vector<string> expected{ "first", "second", "third" };
  ASSERT_EQ(supervisor.manifest.size(), 3);
  ASSERT_EQ(supervisor.manifest, expected);
}


TEST_F(SuperNodeMediatorTest, parseManifest_TrailingCommaIsIgnored)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "first,");
  vector<string> expected{ "first"};
  EXPECT_EQ(supervisor.manifest.size(), 1);
  EXPECT_EQ(supervisor.manifest, expected);
}

TEST_F(SuperNodeMediatorTest, parseManifest_EmptyCommaInMiddleIsIgnored)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "first,,second");
  vector<string> expected{ "first","second"};
  EXPECT_EQ(supervisor.manifest.size(), 2);
  EXPECT_EQ(supervisor.manifest, expected);
}

TEST_F(SuperNodeMediatorTest, parseManifest_Single)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "first");
  vector<string> expected{ "first" };
  ASSERT_EQ(supervisor.manifest.size(), 1);
  ASSERT_EQ(supervisor.manifest, expected);
}
TEST_F(SuperNodeMediatorTest, parseManifest_Double)
{
  SuperNodeMediator::Supervisor supervisor;
  superNodeMediator.parseManifest(supervisor, "first,second");
  vector<string> expected{ "first", "second" };
  ASSERT_EQ(supervisor.manifest.size(), 2);
  ASSERT_EQ(supervisor.manifest, expected);
}

TEST_F(SuperNodeMediatorTest, nodesOnlineCount_EmptyNodesIsZero)
{
  SuperNodeMediator::Supervisor supervisor;
  int count = superNodeMediator.nodesOnlineCount(supervisor);
  ASSERT_EQ(count, 0);
}

TEST_F(SuperNodeMediatorTest, nodesOnlineCount_OneNotOnlineIsZero)
{
  SuperNodeMediator::Supervisor supervisor;
  SuperNodeMediator::SuperNodeInfo node;
  node.online = false;
  supervisor.nodes.insert({ "whatever", node });
  int count = superNodeMediator.nodesOnlineCount(supervisor);
  ASSERT_EQ(count, 0);
}

TEST_F(SuperNodeMediatorTest, nodesOnlineCount_OneOnlineIsOne)
{
  SuperNodeMediator::Supervisor supervisor;
  SuperNodeMediator::SuperNodeInfo node;
  node.online = true;
  supervisor.nodes.insert({ "whatever", node });
  int count = superNodeMediator.nodesOnlineCount(supervisor);
  ASSERT_EQ(count, 1);
}

/**Combination of each of manifested and online testing both methods*/
TEST_F(SuperNodeMediatorTest, nodesOnlineCount_ManifestedOnlinedMixed)
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


TEST_F(SuperNodeMediatorTest, manifestedNodesNotOnline_RemovesNonManifestedNotOnline)
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

TEST_F(SuperNodeMediatorTest, setControllerState)
{
  SuperNodeMediator::Supervisor supervisor;

  ControllerState state = ControllerState::COMPLETED;
  superNodeMediator.setControllerState(supervisor, state);

  ASSERT_EQ(supervisor.last_controller_state_received, state);
}

TEST_F(SuperNodeMediatorTest, setOperatorCommand)
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

TEST_F(SuperNodeMediatorTest, getStateTransition_BootingToReady)
{
  ASSERT_getStateTransition(SuperState::BOOTING, SuperState::READY);
}

TEST_F(SuperNodeMediatorTest, getStateTransition_ReadyToArming)
{
  ASSERT_getStateTransition(SuperState::READY, SuperState::ARMING, OperatorCommand::ARM);
}

TEST_F(SuperNodeMediatorTest, getStateTransition_ArmingToArmed)
{
  ASSERT_getStateTransition(SuperState::ARMING, SuperState::ARMED);
}

TEST_F(SuperNodeMediatorTest, getStateTransition_ArmedToAutoWhenOperatorSendsLaunch)
{
  ASSERT_getStateTransition(SuperState::ARMED, SuperState::AUTO, OperatorCommand::LAUNCH);
}

TEST_F(SuperNodeMediatorTest, getStateTransition_ArmedToDisarmingWhenOperatorSendsCancel)
{
  ASSERT_getStateTransition(SuperState::ARMED, SuperState::DISARMING, OperatorCommand::CANCEL);
}

TEST_F(SuperNodeMediatorTest, getStateTransition_AutoToDisarmingWhenControllerStateIsCompleted)
{
  ASSERT_getStateTransition(SuperState::AUTO, SuperState::DISARMING, (OperatorCommand)NULL, ControllerState::COMPLETED);
}

TEST_F(SuperNodeMediatorTest, getStateTransition_DisarmingToReady)
{
  ASSERT_getStateTransition(SuperState::DISARMING, SuperState::READY);
}

TEST_F(SuperNodeMediatorTest, getStateTransition_AutoToManualWhenOperatorSendsManual)
{
  ASSERT_getStateTransition(SuperState::AUTO, SuperState::MANUAL, OperatorCommand::MANUAL);
}


TEST_F(SuperNodeMediatorTest, isValid_stateTransition)
{
  SuperNodeMediator::StateTransition t;
  ASSERT_FALSE(superNodeMediator.transitionIsValid(t));

  t.to_state = SuperState::AUTO;
  ASSERT_TRUE(superNodeMediator.transitionIsValid(t));
}

TEST_F(SuperNodeMediatorTest, invalidTransition)
{
  SuperNodeMediator::StateTransition t = superNodeMediator.invalidTransition();
  ASSERT_FALSE(superNodeMediator.transitionIsValid(t));
}

void assertPlatformVariant(SuperNodeMediator::PlatformVariant &expected, SuperNodeMediator::PlatformVariant &actual)
{
  ASSERT_EQ(expected.maker,actual.maker);
  ASSERT_EQ(expected.model,actual.model);
  ASSERT_EQ(expected.app,actual.app);
}

TEST_F(SuperNodeMediatorTest, platformConfigToVariant_makerOnly)
{
  SuperNodeMediator::PlatformVariant expected;
  SuperNodeMediator::PlatformVariant actual = superNodeMediator.platformConfigToVariant("dji");
  expected.maker="dji";
  assertPlatformVariant(expected,actual);
}

TEST_F(SuperNodeMediatorTest, platformConfigToVariant_makerModel)
{
  SuperNodeMediator::PlatformVariant expected;
  SuperNodeMediator::PlatformVariant actual = superNodeMediator.platformConfigToVariant("dji_m300");
  expected.maker="dji";
  expected.model="m300";
  assertPlatformVariant(expected,actual);}

TEST_F(SuperNodeMediatorTest, platformConfigToVariant_makerModelApp)
{
  SuperNodeMediator::PlatformVariant expected;
  SuperNodeMediator::PlatformVariant actual = superNodeMediator.platformConfigToVariant("dji_m300_bridge");
  expected.maker="dji";
  expected.model="m300";
  expected.app="bridge";
  assertPlatformVariant(expected,actual);
}

TEST_F(SuperNodeMediatorTest, platformVariantToConfig_empty)
{
  SuperNodeMediator::PlatformVariant variant;
  ASSERT_EQ(superNodeMediator.platformVariantToConfig(variant),"");
}
TEST_F(SuperNodeMediatorTest, platformVariantToConfig_makerOnly)
{
  SuperNodeMediator::PlatformVariant variant;
  variant.maker="dji";
  ASSERT_EQ(superNodeMediator.platformVariantToConfig(variant),"dji");
}

TEST_F(SuperNodeMediatorTest, platformVariantToConfig_makerModel)
{
  SuperNodeMediator::PlatformVariant variant;
  variant.maker="dji";
  variant.model="m300";
  ASSERT_EQ(superNodeMediator.platformVariantToConfig(variant),"dji_m300");
}

TEST_F(SuperNodeMediatorTest, platformVariantToConfig_makerModelApp)
{
  SuperNodeMediator::PlatformVariant variant;
  variant.maker="dji";
  variant.model="m300";
  variant.app="bridge";
  ASSERT_EQ(superNodeMediator.platformVariantToConfig(variant),"dji_m300_bridge");
}


TEST_F(SuperNodeMediatorTest, isCorrectPlatform_sameConfigIsTrue)
{
  std::string config = "dji_m300_bridge";
  SuperNodeMediator::PlatformVariant expected = superNodeMediator.platformConfigToVariant(config);
  SuperNodeMediator::PlatformVariant actual = superNodeMediator.platformConfigToVariant(config);
  ASSERT_TRUE(superNodeMediator.isCorrectPlatform(expected,actual));
}

TEST_F(SuperNodeMediatorTest, isCorrectPlatform_missingAppIsFalse)
{
  SuperNodeMediator::PlatformVariant expected = superNodeMediator.platformConfigToVariant("dji_m300_bridge");
  SuperNodeMediator::PlatformVariant actual = superNodeMediator.platformConfigToVariant("dji_m300");
  ASSERT_FALSE(superNodeMediator.isCorrectPlatform(expected,actual));
}

/** Demonstrate any drone could work as long as it satisfies the app requirements */
TEST_F(SuperNodeMediatorTest, isCorrectPlatform_appOnlyIsTrue)
{
  SuperNodeMediator::PlatformVariant expected;
  expected.app = "bridge";
  SuperNodeMediator::PlatformVariant actual = superNodeMediator.platformConfigToVariant("dji_m300_bridge");
  ASSERT_TRUE(superNodeMediator.isCorrectPlatform(expected,actual));
}

TEST_F(SuperNodeMediatorTest, isCorrectPlatform_wrongAppIsFalse)
{
  SuperNodeMediator::PlatformVariant expected;
  expected.app = "bridge";
  SuperNodeMediator::PlatformVariant actual = superNodeMediator.platformConfigToVariant("dji_m300_culvert");
  ASSERT_FALSE(superNodeMediator.isCorrectPlatform(expected,actual));
}


TEST_F(SuperNodeMediatorTest, isCorrectPlatform_emptyRequirementsIsTrue)
{
  SuperNodeMediator::PlatformVariant expected;
  SuperNodeMediator::PlatformVariant actual = superNodeMediator.platformConfigToVariant("dji_m300_culvert");
  ASSERT_TRUE(superNodeMediator.isCorrectPlatform(expected,actual));
}

