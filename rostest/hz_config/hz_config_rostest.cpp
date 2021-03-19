#include <am_rostest_lib/am_rostest.h>
#include <super_lib/am_stat_reset.h>

class HzConfigTest : public RostestBase, am::AMLifeCycle
{
protected:

  AMStatReset hz_stats_target_ten_ = AMStatReset("default_target_ten", "Test HZ");
  AMStatReset hz_stats_target_one_hundred_ = AMStatReset("default_target_one_hundred", "Test HZ");
  AMStatReset hz_stats_config_target_ten_ = AMStatReset("config_target_ten", "Test HZ");
  AMStatReset hz_stats_config_no_target_ = AMStatReset("config_no_target", "Test HZ");
  HzConfigTest() : RostestBase() {
    configureHzStats(hz_stats_target_ten_,10);
    configureHzStats(hz_stats_target_one_hundred_,100);
    configureHzStats(hz_stats_config_target_ten_,999);
    configureHzStats(hz_stats_config_no_target_,999);
  }

};

TEST_F(HzConfigTest, test_default_target_ten)
{
  ASSERT_EQ(8,hz_stats_target_ten_.getMinError());
  ASSERT_EQ(9,hz_stats_target_ten_.getMinWarn());
  ASSERT_EQ(11,hz_stats_target_ten_.getMaxWarn());
  ASSERT_EQ(12,hz_stats_target_ten_.getMaxError());
}


TEST_F(HzConfigTest, hz_stats_config_target_ten_)
{
  ASSERT_EQ(8,hz_stats_config_target_ten_.getMinError());
  ASSERT_EQ(9,hz_stats_config_target_ten_.getMinWarn());
  ASSERT_EQ(11,hz_stats_config_target_ten_.getMaxWarn());
  ASSERT_EQ(12,hz_stats_config_target_ten_.getMaxError());
}

TEST_F(HzConfigTest, config_no_target)
{
  ASSERT_EQ(8,hz_stats_config_no_target_.getMinError());
  ASSERT_EQ(9,hz_stats_config_no_target_.getMinWarn());
  ASSERT_EQ(11,hz_stats_config_no_target_.getMaxWarn());
  ASSERT_EQ(12,hz_stats_config_no_target_.getMaxError());
}

TEST_F(HzConfigTest, default_target_one_hundred)
{
  ASSERT_EQ(90,hz_stats_target_one_hundred_.getMinError());
  ASSERT_EQ(95,hz_stats_target_one_hundred_.getMinWarn());
  ASSERT_EQ(105,hz_stats_target_one_hundred_.getMaxWarn());
  ASSERT_EQ(110,hz_stats_target_one_hundred_.getMaxError());
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}