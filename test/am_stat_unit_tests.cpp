#include <gtest/gtest.h>
#include <super_lib/am_stat.h>

using namespace am;
using namespace std;

class AMStatTest : public ::testing::Test
{
protected:
  AMStat stat_;
  const string short_name = "s1";
  const string long_name = "Stat 1";
  const uint32_t max_warn = 5;
  const uint32_t max_error = 10;
  AMStatTest(): stat_("s1", "Stat 1", 5, 10) {}
};

TEST_F(AMStatTest, compoundStatus)
{
  LifeCycleStatus status = LifeCycleStatus::OK;

  AMStat::compoundStatus(status, LifeCycleStatus::OK);
  ASSERT_EQ(status, LifeCycleStatus::OK);

  AMStat::compoundStatus(status, LifeCycleStatus::WARN);
  ASSERT_EQ(status, LifeCycleStatus::WARN);

  AMStat::compoundStatus(status, LifeCycleStatus::OK);
  ASSERT_EQ(status, LifeCycleStatus::WARN);

  AMStat::compoundStatus(status, LifeCycleStatus::ERROR);
  ASSERT_EQ(status, LifeCycleStatus::ERROR);

  AMStat::compoundStatus(status, LifeCycleStatus::OK);
  ASSERT_EQ(status, LifeCycleStatus::ERROR);

  AMStat::compoundStatus(status, LifeCycleStatus::WARN);
  ASSERT_EQ(status, LifeCycleStatus::ERROR);
}

TEST_F(AMStatTest, operators)
{
  const int VALUE = 1;

  stat_ = VALUE;
  ASSERT_EQ(stat_.getCount(), VALUE);
  stat_++;
  ASSERT_EQ(stat_.getCount(), VALUE + 1);
  stat_ += 2;
  ASSERT_EQ(stat_.getCount(), VALUE + 3);
}

TEST_F(AMStatTest, reset)
{
  stat_ = 3;
  stat_.reset();
  ASSERT_EQ(stat_.getCount(), 3);
}

TEST_F(AMStatTest, getShortName)
{
  ASSERT_EQ(short_name, stat_.getShortName());
}

TEST_F(AMStatTest, getLongName)
{
  ASSERT_EQ(long_name, stat_.getLongName());
}

TEST_F(AMStatTest, add)
{
  stat_ = 3;
  stat_.add(2);
  ASSERT_EQ(stat_.getCount(), 5);
}