#include <gtest/gtest.h>  // googletest header file

#include <string>
using std::string;

const char* actualValTrue = "hello gtest";
const char* actualValFalse = "hello world";
const char* expectVal = "hello gtest";

TEST(StrCompare, CStrEqual)
{
  EXPECT_STREQ(expectVal, actualValTrue);
}

TEST(StrCompare, CStrEqual2)
{
  EXPECT_STREQ(expectVal, actualValTrue);
}

TEST(StrCompare, CStrEqual3)
{
  EXPECT_STREQ(expectVal, actualValTrue);
}

TEST(StrCompare, CStrNotEqual)
{
  EXPECT_STREQ(expectVal, actualValFalse);
}

TEST(StrCompare, CStrNotEqual4)
{
  EXPECT_STREQ(expectVal, actualValFalse);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}