#include <gtest/gtest.h>  // googletest header file
#include <am_super/super_node_mediator.h>

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
  std::string name="node1";
  SuperNodeMediator::SuperNodeInfo nodeInfo = superNodeMediator.initializeManifestedNode(name);
  ASSERT_EQ(nodeInfo.name,name) << "Safety check for index of a string";  
  ASSERT_EQ(nodeInfo.pid,-1) << "Doesn't really matter...references purposes only";
  ASSERT_EQ(nodeInfo.online,false) << "Initialized, but not yet reported anything";
  ASSERT_EQ(nodeInfo.manifested,true) << "Declared in the manifest";
  ASSERT_EQ(nodeInfo.state,LifeCycleState::UNCONFIGURED) << "Beginning of state lifecycle";
  ASSERT_EQ(nodeInfo.status,LifeCycleStatus::OK) << "All is good until reported otherwise";
}
