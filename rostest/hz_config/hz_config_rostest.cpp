#include <am_rostest_lib/am_rostest.h>
#include <super_lib/am_stat_reset.h>

class HzConfigTest : public RostestBase, am::AMLifeCycle
{
protected:

  AMStatReset hz_stats_target_one_ = AMStatReset("target_one", "target_one");
  AMStatReset hz_stats_target_ten_ = AMStatReset("target_ten", "target_ten");
  AMStatReset hz_stats_target_one_hundred_ = AMStatReset("target_one_hundred", "target_one_hundred");
  AMStatReset hz_stats_no_target_ = AMStatReset("no_target", "no_target");
  AMStatReset hz_stats_all_provided_ = AMStatReset("all_provided", "all_provided");
  AMStatReset hz_stats_target_one_hundred_min_error_10_ = AMStatReset("target_one_hundred_min_error_10", "target_one_hundred_min_error_10");
  AMStatReset hz_stats_min_only_ = AMStatReset("min_only", "min_only");
  AMStatReset hz_stats_max_only_ = AMStatReset("max_only", "max_only");
  AMStat stat_none_ = AMStat("stat_none", "stat_none");
  AMStat stat_none_with_default_max_ = AMStat("stat_none_with_default_max", "stat_none_with_default_max",15,20);

  HzConfigTest() : RostestBase() {
    configureHzStat(hz_stats_target_ten_);
    configureHzStat(hz_stats_target_one_);
    configureHzStat(hz_stats_target_one_hundred_);
    configureHzStat(hz_stats_no_target_);
    configureHzStat(hz_stats_all_provided_);
    configureHzStat(hz_stats_target_one_hundred_min_error_10_);
    configureHzStat(hz_stats_min_only_);
    configureHzStat(hz_stats_max_only_);
    configureStat(stat_none_);
    configureStat(stat_none_with_default_max_);
  }

};

TEST_F(HzConfigTest, target_ten)
{
  ASSERT_EQ(8,hz_stats_target_ten_.getMinError());
  ASSERT_EQ(9,hz_stats_target_ten_.getMinWarn());
  ASSERT_EQ(11,hz_stats_target_ten_.getMaxWarn());
  ASSERT_EQ(12,hz_stats_target_ten_.getMaxError());
  ASSERT_TRUE(hz_stats_target_ten_.isValidatingMax());
  ASSERT_TRUE(hz_stats_target_ten_.isValidatingMin());
}
TEST_F(HzConfigTest, target_one)
{
  ASSERT_EQ(0,hz_stats_target_one_.getMinError()) << "Shouldn't go below zero";
  ASSERT_EQ(0,hz_stats_target_one_.getMinWarn()) << "shouldn't go below zero";
  ASSERT_EQ(2,hz_stats_target_one_.getMaxWarn());
  ASSERT_EQ(3,hz_stats_target_one_.getMaxError());
  ASSERT_TRUE(hz_stats_target_one_.isValidatingMax());
  ASSERT_TRUE(hz_stats_target_one_.isValidatingMin());
}

TEST_F(HzConfigTest, all_provided)
{
  ASSERT_EQ(11,hz_stats_all_provided_.getMinError());
  ASSERT_EQ(22,hz_stats_all_provided_.getMinWarn());
  ASSERT_EQ(33,hz_stats_all_provided_.getMaxWarn());
  ASSERT_EQ(44,hz_stats_all_provided_.getMaxError());
  ASSERT_TRUE(hz_stats_all_provided_.isValidatingMax());
  ASSERT_TRUE(hz_stats_all_provided_.isValidatingMin());
}

TEST_F(HzConfigTest, no_target)
{
  ASSERT_FALSE(hz_stats_no_target_.isValidatingMax());
  ASSERT_FALSE(hz_stats_no_target_.isValidatingMin());
}

TEST_F(HzConfigTest, stat_none)
{
  ASSERT_FALSE(stat_none_.isValidatingMax());
  ASSERT_FALSE(stat_none_.isValidatingMin());
}
TEST_F(HzConfigTest, stat_none_with_default_max)
{
  ASSERT_TRUE(stat_none_with_default_max_.isValidatingMax());
  ASSERT_FALSE(stat_none_with_default_max_.isValidatingMin());
  //magic numbers provided upon initialization of stat
  ASSERT_EQ(stat_none_with_default_max_.getMaxWarn(),15);
  ASSERT_EQ(stat_none_with_default_max_.getMaxError(),20);
}

TEST_F(HzConfigTest, min_only)
{
  ASSERT_EQ(77,hz_stats_min_only_.getMinError());
  ASSERT_EQ(88,hz_stats_min_only_.getMinWarn());
  ASSERT_FALSE(hz_stats_min_only_.isValidatingMax());
  ASSERT_TRUE(hz_stats_min_only_.isValidatingMin());
}
TEST_F(HzConfigTest, max_only)
{
  ASSERT_EQ(63,hz_stats_max_only_.getMaxError());
  ASSERT_EQ(51,hz_stats_max_only_.getMaxWarn());
  ASSERT_TRUE(hz_stats_max_only_.isValidatingMax());
  ASSERT_FALSE(hz_stats_max_only_.isValidatingMin());
}

TEST_F(HzConfigTest, target_one_hundred)
{
  ASSERT_EQ(90,hz_stats_target_one_hundred_.getMinError());
  ASSERT_EQ(95,hz_stats_target_one_hundred_.getMinWarn());
  ASSERT_EQ(105,hz_stats_target_one_hundred_.getMaxWarn());
  ASSERT_EQ(110,hz_stats_target_one_hundred_.getMaxError());
  ASSERT_TRUE(hz_stats_target_one_hundred_.isValidatingMax());
  ASSERT_TRUE(hz_stats_target_one_hundred_.isValidatingMin());
}

TEST_F(HzConfigTest, target_one_hundred_min_error_10)
{
  ASSERT_EQ(10,hz_stats_target_one_hundred_min_error_10_.getMinError());
  ASSERT_EQ(95,hz_stats_target_one_hundred_min_error_10_.getMinWarn());
  ASSERT_EQ(105,hz_stats_target_one_hundred_min_error_10_.getMaxWarn());
  ASSERT_EQ(110,hz_stats_target_one_hundred_min_error_10_.getMaxError());
  ASSERT_TRUE(hz_stats_target_one_hundred_min_error_10_.isValidatingMax());
  ASSERT_TRUE(hz_stats_target_one_hundred_min_error_10_.isValidatingMin());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}