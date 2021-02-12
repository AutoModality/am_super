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

public:
  AMStatReset(const std::string& short_name, const std::string& long_name) : AMStat(short_name, long_name){}

  AMStatReset(const std::string& short_name, const std::string& long_name, uint32_t max_warn, uint32_t max_error)
    : AMStat(short_name, long_name, max_warn, max_error){}

  AMStatReset(const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
              uint32_t max_warn, uint32_t max_error)
    : AMStat(short_name, long_name,min_error,min_warn, max_warn, max_error){}

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
