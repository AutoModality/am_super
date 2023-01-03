#include <am_rostest_lib/am_rostest.h>

class ParamTest : public RostestBase, am::AMLifeCycle
{
protected:
  std::string relative_param_;
  bool relative_result_;
  std::string absolute_param_;
  bool absolute_result_;
  std::string default_param_;
  bool default_result_;
  std::string default_value_ = "default_value";

  ParamTest() : RostestBase() {
      relative_result_=param("relative",relative_param_,default_value_);
      absolute_result_=param("/some_other_node/absolute",absolute_param_,default_value_);
      default_result_=param("/some_node/not_in_file",default_param_,default_value_);
  }

};

TEST_F(ParamTest, test_params)
{
  ASSERT_EQ(relative_param_,"relative_value");
  ASSERT_TRUE(relative_result_) << "true since it found a configuration";

  ASSERT_EQ(absolute_param_,"absolute_value");
  ASSERT_TRUE(absolute_result_) << "true since it found a configuration";

  ASSERT_EQ(default_param_,default_value_);
  ASSERT_FALSE(default_result_) << "false since no configuration found";

}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());

  return RUN_ALL_TESTS();
}