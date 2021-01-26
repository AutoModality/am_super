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

TEST_F(AMStatTest, constructor)
{
  ASSERT_EQ(stat_.getCount(), 0);
  ASSERT_EQ(stat_.getShortName(), short_name);
  ASSERT_EQ(stat_.getLongName(), long_name);
  ASSERT_EQ(stat_.getMaxWarn(), max_warn);
  ASSERT_EQ(stat_.getMaxError(), max_error);
}

TEST_F(AMStatTest, process_OK)
{
  LifeCycleStatus status = stat_.process(1, 1);
  ASSERT_EQ(status, LifeCycleStatus::OK);
}

TEST_F(AMStatTest, process_WARN)
{
  stat_ = 6;
  LifeCycleStatus status = stat_.process(1, 1);
  ASSERT_EQ(status, LifeCycleStatus::WARN);
}