#include <gtest/gtest.h>  // googletest header file
#include <am_super/super_node_mediator.h>

using namespace std;
using namespace am;

SuperNodeMediator superNodeMediator;

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

void ASSERT_CHECK(std::function<bool(SuperNodeMediator::SuperNodeInfo&)> check, LifeCycleState state, bool expected)
{
  SuperNodeMediator::SuperNodeInfo info;
  info.state = state;
  ASSERT_EQ(check(info), expected) << "For state: " + std::to_string((int)state);
}

TEST(Node, checkReadyForConfigureState_All)
{
  std::function<bool(SuperNodeMediator::SuperNodeInfo&)> function = SuperNodeMediator::checkReadyForConfigureState;
  ASSERT_CHECK(function, LifeCycleState::INVALID, false);
  ASSERT_CHECK(function, LifeCycleState::UNCONFIGURED, true);
  ASSERT_CHECK(function, LifeCycleState::INACTIVE, true);
  ASSERT_CHECK(function, LifeCycleState::ACTIVE, true);
  ASSERT_CHECK(function, LifeCycleState::FINALIZED, false);
  ASSERT_CHECK(function, LifeCycleState::CONFIGURING, false);
  ASSERT_CHECK(function, LifeCycleState::CLEANING_UP, false);
  ASSERT_CHECK(function, LifeCycleState::SHUTTING_DOWN, false);
  ASSERT_CHECK(function, LifeCycleState::ACTIVATING, false);
  ASSERT_CHECK(function, LifeCycleState::DEACTIVATING, false);
}

TEST(Node, checkReadyForActivateState_All)
{
  std::function<bool(SuperNodeMediator::SuperNodeInfo&)> function = SuperNodeMediator::checkReadyForActivateState;
  ASSERT_CHECK(function, LifeCycleState::INVALID, false);
  ASSERT_CHECK(function, LifeCycleState::UNCONFIGURED, false);
  ASSERT_CHECK(function, LifeCycleState::INACTIVE, true);
  ASSERT_CHECK(function, LifeCycleState::ACTIVE, true);
  ASSERT_CHECK(function, LifeCycleState::FINALIZED, false);
  ASSERT_CHECK(function, LifeCycleState::CONFIGURING, false);
  ASSERT_CHECK(function, LifeCycleState::CLEANING_UP, false);
  ASSERT_CHECK(function, LifeCycleState::SHUTTING_DOWN, false);
  ASSERT_CHECK(function, LifeCycleState::ACTIVATING, false);
  ASSERT_CHECK(function, LifeCycleState::DEACTIVATING, false);
}
TEST(Node, checkActivateState_All)
{
  std::function<bool(SuperNodeMediator::SuperNodeInfo&)> function = SuperNodeMediator::checkActivateState;
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

void assertAllManifestedNodesCheck(bool expected_success, SuperNodeMediator::SuperNodeInfo node, bool check_result,
                                   string expected_error_code = "")
{
  std::function<bool(SuperNodeMediator::SuperNodeInfo&)> check = [check_result](SuperNodeMediator::SuperNodeInfo&) {
    return check_result;
  };
  SuperNodeMediator::Supervisor supervisor;
  string expected_node_name = "test-node";
  node.name = expected_node_name;
  supervisor.nodes.insert({ node.name, node });
  pair<bool, map<string, string>> result = superNodeMediator.allManifestedNodesCheck(supervisor, check);
  bool success = get<0>(result);
  ASSERT_EQ(success, expected_success);
  if (!expected_error_code.empty())
  {
    map<string, string> error_messages = get<1>(result);
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