#include <gtest/gtest.h>
#include <super_lib/am_stat.h>
#include <super_lib/am_stat_reset.h>
#include <super_lib/am_stat_list.h>

using namespace am;
using namespace std;

const string short_name = "s1";
const string long_name = "Stat 1";

const uint32_t min_error = 2;
const uint32_t min_warn = 3;
const uint32_t max_warn = 5;
const uint32_t max_error = 10;
class AMStatTest : public ::testing::Test
{
protected:
  AMStat stat_max_;
  AMStat stat_min_max_;
  AMStatReset stat_reset_;
  AMStatList stat_list_;

  AMStatTest(): 
    stat_max_(short_name, long_name, max_warn, max_error), 
    stat_min_max_(short_name, long_name, min_error,min_warn, max_warn, max_error), 
    stat_reset_(short_name, long_name, max_warn, max_error) 
    {};
};

TEST_F(AMStatTest, getShortName_constructed)
{
  ASSERT_EQ(short_name, stat_max_.getShortName());
  ASSERT_EQ(short_name, stat_min_max_.getShortName());
  ASSERT_EQ(short_name, stat_reset_.getShortName());
  //get methods will ensure constructor is working
}

TEST_F(AMStatTest, getLongName_constructed)
{
  ASSERT_EQ(long_name, stat_max_.getLongName());
  ASSERT_EQ(long_name, stat_min_max_.getLongName());
  ASSERT_EQ(long_name, stat_reset_.getLongName());
}

TEST_F(AMStatTest, getMaxWarn_constructed)
{
  ASSERT_EQ(max_warn, stat_max_.getMaxWarn());
  ASSERT_EQ(max_warn, stat_min_max_.getMaxWarn());
  ASSERT_EQ(max_warn, stat_reset_.getMaxWarn());  
}

TEST_F(AMStatTest, getMaxError_constructed)
{
  ASSERT_EQ(max_error, stat_max_.getMaxError());
  ASSERT_EQ(max_error, stat_min_max_.getMaxError());
  ASSERT_EQ(max_error, stat_reset_.getMaxError());
}

TEST_F(AMStatTest, isValidatingMax_constructed)
{
  ASSERT_TRUE(stat_max_.isValidatingMax());
  ASSERT_TRUE(stat_min_max_.isValidatingMax());
  ASSERT_TRUE(stat_reset_.isValidatingMax()); 
}

TEST_F(AMStatTest, isValidatingMin_constructed)
{
  ASSERT_FALSE(stat_max_.isValidatingMin());
  ASSERT_TRUE(stat_min_max_.isValidatingMin());
  ASSERT_FALSE(stat_reset_.isValidatingMin()); 
}

TEST_F(AMStatTest, getValue_constructed)
{
  //0 is initial value
  ASSERT_EQ(0, stat_max_.getValue());
}

TEST_F(AMStatTest, setMaxWarn)
{
  int val = max_warn + 1;
  stat_max_.setMaxWarn(val);
  ASSERT_EQ(val, stat_max_.getMaxWarn());
}

TEST_F(AMStatTest, setMaxError)
{
  int val = max_error + 1;
  stat_max_.setMaxError(val);
  ASSERT_EQ(val, stat_max_.getMaxError());
}

TEST_F(AMStatTest, setWarnError)
{
  int warn = max_warn + 1;
  int error = max_error + 1;
  stat_max_.setWarnError(warn, error);
  ASSERT_EQ(warn, stat_max_.getMaxWarn());
  ASSERT_EQ(error, stat_max_.getMaxError());
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

  ASSERT_FALSE(stat_max_.isSampleReceived());
  stat_max_ = VALUE;
  ASSERT_TRUE(stat_max_.isSampleReceived());
  ASSERT_EQ(stat_max_.getValue(), VALUE);

  ASSERT_FALSE(stat_reset_.isSampleReceived());
  stat_reset_ = VALUE;
  ASSERT_TRUE(stat_reset_.isSampleReceived());
  ASSERT_EQ(stat_reset_.getValue(),VALUE);

  ASSERT_FALSE(stat_min_max_.isSampleReceived());
  stat_min_max_ = VALUE;
  ASSERT_TRUE(stat_min_max_.isSampleReceived());
  ASSERT_EQ(stat_min_max_.getValue(),VALUE);

  stat_max_++;
  ASSERT_EQ(stat_max_.getValue(), VALUE + 1);
  stat_reset_++;
  ASSERT_EQ(stat_reset_.getValue(), VALUE + 1);
  stat_min_max_++;
  ASSERT_EQ(stat_min_max_.getValue(), VALUE + 1);

  stat_max_ += 2;
  ASSERT_EQ(stat_max_.getValue(), VALUE + 3);
  stat_reset_ += 2;
  ASSERT_EQ(stat_reset_.getValue(), VALUE + 3);
  stat_min_max_ += 2;
  ASSERT_EQ(stat_min_max_.getValue(), VALUE + 3);
}

TEST_F(AMStatTest, isSampleReceived_operatorPlusPlus)
{

  ASSERT_FALSE(stat_max_.isSampleReceived());
  stat_max_++;
  ASSERT_TRUE(stat_max_.isSampleReceived());

  ASSERT_FALSE(stat_reset_.isSampleReceived());
  stat_reset_++;
  ASSERT_TRUE(stat_reset_.isSampleReceived());

  ASSERT_FALSE(stat_min_max_.isSampleReceived());
  stat_min_max_++;
  ASSERT_TRUE(stat_min_max_.isSampleReceived());

}

TEST_F(AMStatTest, isSampleRequired)
{
  ASSERT_FALSE(stat_max_.isSampleRequired());
  ASSERT_TRUE(stat_reset_.isSampleRequired());
}

TEST_F(AMStatTest, isSampleReceived_operatorPlusEquals)
{

  ASSERT_FALSE(stat_max_.isSampleReceived());
  stat_max_+=1;
  ASSERT_TRUE(stat_max_.isSampleReceived());

  ASSERT_FALSE(stat_reset_.isSampleReceived());
  stat_reset_+=1;
  ASSERT_TRUE(stat_reset_.isSampleReceived());

  ASSERT_FALSE(stat_min_max_.isSampleReceived());
  stat_min_max_+=1;
  ASSERT_TRUE(stat_min_max_.isSampleReceived());

}


TEST_F(AMStatTest, reset)
{
  stat_max_ = 3;
  stat_max_.reset();
  ASSERT_EQ(stat_max_.getValue(), 3);
}

TEST_F(AMStatTest, add)
{
  stat_max_ = 3;
  stat_max_.add(2);
  ASSERT_EQ(stat_max_.getValue(), 5);
}

TEST_F(AMStatTest, stat_list_hasStats)
{
  ASSERT_FALSE(stat_list_.hasStats());
  stat_list_.add(&stat_max_);
  ASSERT_TRUE(stat_list_.hasStats());
}