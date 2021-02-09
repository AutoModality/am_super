#include <gtest/gtest.h>
#include <super_lib/am_stat.h>

using namespace am;
using namespace std;

class AMStatTest : public ::testing::Test
{
protected:
  AMStat stat_;
  string short_name = "s1";
  string long_name = "Stat 1";
  uint32_t max_warn = 5;
  uint32_t max_error = 10;
  AMStatTest(): stat_("s1", "Stat 1", 5, 10) {}
};

TEST_F(AMStatTest, compoundStatus)
{

}