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
