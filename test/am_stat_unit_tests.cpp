#include <gtest/gtest.h>
#include <super_lib/am_stat.h>
#include <super_lib/am_stat_reset.h>
#include <super_lib/am_stat_list.h>

using namespace am;
using namespace std;

const string short_name = "s1";
const string long_name = "Stat 1";
const uint32_t max_warn = 5;
const uint32_t max_error = 10;
class AMStatTest : public ::testing::Test
{
protected:
  AMStat stat_;
  AMStatReset stat_reset_;
  AMStatList stat_list_;

  AMStatTest(): 
    stat_(short_name, long_name, max_warn, max_error), 
    stat_reset_(short_name, long_name, max_warn, max_error) 
    {};
};

TEST_F(AMStatTest, getShortName_constructed)
{
  ASSERT_EQ(short_name, stat_.getShortName());
  ASSERT_EQ(short_name, stat_reset_.getShortName());
  //get methods will ensure constructor is working
}

TEST_F(AMStatTest, getLongName_constructed)
{
  ASSERT_EQ(long_name, stat_.getLongName());
  ASSERT_EQ(long_name, stat_reset_.getLongName());
}

TEST_F(AMStatTest, getMaxWarn_constructed)
{
  ASSERT_EQ(max_warn, stat_.getMaxWarn());
  ASSERT_EQ(max_warn, stat_reset_.getMaxWarn());
}

TEST_F(AMStatTest, getMaxError_constructed)
{
  ASSERT_EQ(max_error, stat_.getMaxError());
  ASSERT_EQ(max_error, stat_reset_.getMaxError());
}


TEST_F(AMStatTest, getValue_constructed)
{
  //0 is initial value
  ASSERT_EQ(0, stat_.getValue());
}

TEST_F(AMStatTest, setMaxWarn)
{
  int val = max_warn + 1;
  stat_.setMaxWarn(val);
  ASSERT_EQ(val, stat_.getMaxWarn());
}

TEST_F(AMStatTest, setMaxError)
{
  int val = max_error + 1;
  stat_.setMaxError(val);
  ASSERT_EQ(val, stat_.getMaxError());
}

TEST_F(AMStatTest, setWarnError)
{
  int warn = max_warn + 1;
  int error = max_error + 1;
  stat_.setWarnError(warn, error);
  ASSERT_EQ(warn, stat_.getMaxWarn());
  ASSERT_EQ(error, stat_.getMaxError());
}

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
  ASSERT_EQ(stat_.getValue(), VALUE);

  stat_reset_ = VALUE;
  ASSERT_EQ(stat_reset_.getValue(),VALUE);

  stat_++;
  ASSERT_EQ(stat_.getValue(), VALUE + 1);
  stat_reset_++;
  ASSERT_EQ(stat_reset_.getValue(), VALUE + 1);

  stat_ += 2;
  ASSERT_EQ(stat_.getValue(), VALUE + 3);

  stat_reset_ += 2;
  ASSERT_EQ(stat_reset_.getValue(), VALUE + 3);


}

TEST_F(AMStatTest, reset)
{
  stat_ = 3;
  stat_.reset();
  ASSERT_EQ(stat_.getValue(), 3);
}

TEST_F(AMStatTest, add)
{
  stat_ = 3;
  stat_.add(2);
  ASSERT_EQ(stat_.getValue(), 5);
}

TEST_F(AMStatTest, stat_list_hasStats)
{
  ASSERT_FALSE(stat_list_.hasStats());
  stat_list_.add(&stat_);
  ASSERT_TRUE(stat_list_.hasStats());
}