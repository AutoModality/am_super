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

  LifeCycleStatus process() override
  {
    LifeCycleStatus status = AMStat::process();

    if (value_ < min_error_)
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, long_name_ << " exceeding min_error: " << value_ << " (min:" << min_error_ << ")");
      compoundStatus(status, LifeCycleStatus::ERROR);
    }
    else if (value_ < min_warn_)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, long_name_ << " exceeding min_warn: " << value_ << " (min:" << min_warn_ << ")");
      compoundStatus(status, LifeCycleStatus::WARN);
    }

    return status;
  }

  void reset() override
  {
    value_ = 0;
  }

  uint32_t getMinError() const
  {
    return min_error_;
  }

  void setMinError(uint32_t minError = 0)
  {
    min_error_ = minError;
  }

  uint32_t getMinWarn() const
  {
    return min_warn_;
  }

  void setMinWarn(uint32_t minWarn = 0)
  {
    min_warn_ = minWarn;
  }
};

};  // namespace am

#endif  // AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_H
