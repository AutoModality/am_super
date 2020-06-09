#include <gtest/gtest.h>  // googletest header file

#include <string>
using std::string;

const char* actualVal2True = "hello gtest";
const char* actualVal2False = "hello world";
const char* expectVal2 = "hello gtest";

TEST(StrCompare, CStrEqual2)
{
  EXPECT_STREQ(expectVal2, actualVal2True);
}
