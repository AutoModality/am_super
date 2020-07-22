#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_RESET_H
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_RESET_H

#include <super_lib/am_stat.h>

namespace am
{
class AMStatReset : public AMStat
{
protected:
  uint32_t min_warn_ = 0;
  uint32_t min_error_ = 0;

private:
  AMStatReset();

public:
  AMStatReset(const std::string& short_name, const std::string& long_name) : AMStat(short_name, long_name)
  {
  }

  AMStatReset(const std::string& short_name, const std::string& long_name, uint32_t max_warn, uint32_t max_error)
    : AMStat(short_name, long_name, max_warn, max_error)
  {
  }

  AMStatReset(const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
              uint32_t max_warn, uint32_t max_error)
    : AMStat(short_name, long_name)
  {
    min_error_ = min_error;
    min_warn_ = min_warn;
    max_warn_ = max_warn;
    max_error_ = max_error;
  }

  LifeCycleStatus process(double warn_throttle_s, double error_throttle_s) override
  {
    LifeCycleStatus status = AMStat::process(warn_throttle_s, error_throttle_s);

    if (value_ < min_error_)
    {
      ROS_ERROR_STREAM_THROTTLE(error_throttle_s, long_name_ << " exceeding min_error: " << value_
                                                             << " (min:" << min_error_ << ")");
      compoundStatus(status, LifeCycleStatus::ERROR);
    }
    else if (value_ < min_warn_)
    {
      ROS_WARN_STREAM_THROTTLE(warn_throttle_s, long_name_ << " exceeding min_warn: " << value_ << " (min:" << min_warn_
                                                           << ")");
      compoundStatus(status, LifeCycleStatus::WARN);
    }

    return status;
  }

  void reset() override
  {
    value_ = 0;
  }

  void setWarnError(uint32_t min_error, uint32_t min_warn, uint32_t max_warn, uint32_t max_error)
  {
    min_error_ = min_error;
    min_warn_ = min_warn;
    max_warn_ = max_warn;
    max_error_ = max_error;
  }

  uint32_t getMinError() const
  {
    return min_error_;
  }

  void setMinError(uint32_t min_error)
  {
    min_error_ = min_error;
  }

  uint32_t getMinWarn() const
  {
    return min_warn_;
  }

  void setMinWarn(uint32_t min_warn)
  {
    min_warn_ = min_warn;
  }
};

};  // namespace am

#endif  // AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_H
