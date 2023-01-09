#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_RESET_H
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_RESET_H

#include <super_lib/am_stat.h>

namespace am
{

/**
 * Additive statics reporting minimum/maximum thresholds where the value is reset 
 * upon every Life Cycle heartbeat of 1 second. This is used for frequency validation
 * like rostopic hz shows.
 *
 */ 
class AMStatReset : public AMStat
{

private:
  AMStatReset();
  void init()
  {
    sample_required_ = true;
  }
public:
  AMStatReset(rclcpp::Node::SharedPtr node, const std::string& short_name, const std::string& long_name) : AMStat(node, short_name, long_name)
  {
    init();
  }

  AMStatReset(rclcpp::Node::SharedPtr node, const std::string& short_name, const std::string& long_name, uint32_t max_warn, uint32_t max_error)
    : AMStat(node, short_name, long_name, max_warn, max_error)
    {
      init();
    }

  AMStatReset(rclcpp::Node::SharedPtr node, const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
              uint32_t max_warn, uint32_t max_error)
    : AMStat(node, short_name, long_name,min_error,min_warn, max_warn, max_error)
    {
      init();
    }

  void reset() override
  {
    value_ = 0;
  }
  
  AMStatReset& operator=(uint32_t assignment)
  {
    sample_received_ = true;
    value_ = assignment;
    return *this;
  }

};


};  // namespace am

#endif  // AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_H
