#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_AVE_H
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_AVE_H

#include <super_lib/am_stat.h>
#include <super_lib/am_stat_reset.h>


namespace am
{
class AMStatAve : public AMStatReset
{
protected:
  uint64_t total_ = 0;
  uint32_t max_ = 0;
  uint32_t min_ = std::numeric_limits<uint32_t>::max();
  uint32_t min_min_warn_ = 0;
  uint32_t min_min_error_ = 0;
  uint32_t max_max_warn_ = std::numeric_limits<uint32_t>::max();
  uint32_t max_max_error_ = std::numeric_limits<uint32_t>::max();

private:
  AMStatAve();

public:
  AMStatAve(const std::string& short_name, const std::string& long_name) : AMStatReset(short_name, long_name)
  {
  }

  AMStatAve(const std::string& short_name, const std::string& long_name, uint32_t max_warn, uint32_t max_error)
    : AMStatReset(short_name, long_name, max_warn, max_error)
  {
  }

  AMStatAve(const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
            uint32_t max_warn, uint32_t max_error)
    : AMStatReset(short_name, long_name, min_error, min_warn, max_warn, max_error)
  {
  }

  AMStatAve(const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
            uint32_t max_warn, uint32_t max_error, uint32_t min_min_error = 0, uint32_t min_min_warn = 0,
            uint32_t max_max_warn = std::numeric_limits<uint32_t>::max(),
            uint32_t max_max_error = std::numeric_limits<uint32_t>::max())
    : AMStatReset(short_name, long_name, min_error, min_warn, max_warn, max_error)
  {
    min_min_error_ = min_min_error;
    min_min_warn_ = min_min_warn;
    max_max_warn_ = max_max_warn;
    max_max_error_ = max_max_error;
  }

  LifeCycleStatus process(double warn_throttle_s, double error_throttle_s) override
  {
    uint32_t ave = getAve();
    LifeCycleStatus status = LifeCycleStatus::OK;

    if (ave > max_error_)
    {
      ROS_ERROR_STREAM_THROTTLE_NAMED(error_throttle_s, am::AM_STATS_TOPIC, long_name_ << " ave exceeding max_error: " << ave
                                      << " (max:" << max_error_ << ")");
      compoundStatus(status, LifeCycleStatus::ERROR);
    }
    else if (ave > max_warn_)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(warn_throttle_s, am::AM_STATS_TOPIC, long_name_ << " ave exceeding max_warn: " << ave
                                                           << " (max:" << max_warn_ << ")");
      compoundStatus(status, LifeCycleStatus::WARN);
    }

    if (ave < min_error_)
    {
      ROS_ERROR_STREAM_THROTTLE_NAMED(error_throttle_s, am::AM_STATS_TOPIC, long_name_ << " ave exceeding min_error: " << ave
                                                             << " (min:" << min_error_ << ")");
      compoundStatus(status, LifeCycleStatus::ERROR);
    }
    else if (ave < min_warn_)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(warn_throttle_s, am::AM_STATS_TOPIC, long_name_ << " ave exceeding min_warn: " << ave
                                                           << " (min:" << min_warn_ << ")");
      compoundStatus(status, LifeCycleStatus::WARN);
    }

    if (min_ < min_min_error_)
    {
      ROS_ERROR_STREAM_THROTTLE_NAMED(error_throttle_s, am::AM_STATS_TOPIC, long_name_ << " min exceeding min_min_error: " << min_
                                                             << " (min:" << min_min_error_ << ")");
      compoundStatus(status, LifeCycleStatus::ERROR);
    }
    else if (min_ < min_min_warn_)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(warn_throttle_s, am::AM_STATS_TOPIC, long_name_ << " min exceeding min_min_warn: " << min_
                                                           << " (min:" << min_min_warn_ << ")");
      compoundStatus(status, LifeCycleStatus::WARN);
    }

    if (max_ > max_max_error_)
    {
      ROS_ERROR_STREAM_THROTTLE_NAMED(error_throttle_s, am::AM_STATS_TOPIC, long_name_ << " max exceeding max_max_error: " << max_
                                                             << " (max:" << max_max_error_ << ")");
      compoundStatus(status, LifeCycleStatus::ERROR);
    }
    else if (max_ > max_max_warn_)
    {
      ROS_WARN_STREAM_THROTTLE_NAMED(warn_throttle_s, am::AM_STATS_TOPIC, long_name_ << " max exceeding max_max_warn: " << max_
                                                           << " (max:" << max_max_warn_ << ")");
      compoundStatus(status, LifeCycleStatus::WARN);
    }

    return status;
  }

  void reset() override
  {
    value_ = 0;
    total_ = 0;
    max_ = 0;
    min_ = std::numeric_limits<uint32_t>::max();
  }

  void add(uint32_t value) override
  {
    total_ += value;
    value_++;
    if (value < min_)
    {
      min_ = value;
    }
    if (value > max_)
    {
      max_ = value;
    }
  }

  uint32_t getAve()
  {
    uint64_t ave_64 = ((float)total_ / (float)value_ + 0.5);
    uint32_t ave_32 = ave_64 > std::numeric_limits<uint32_t>::max() ? std::numeric_limits<uint32_t>::max() : ave_64;
    return ave_32;
  }

  void addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw) override
  {
    dsw.add(long_name_ + " Ave", getAve());
    dsw.add(long_name_ + " Max", getMax());
    dsw.add(long_name_ + " Min", getMin());
  }

  std::string getStrShort() override
  {
    std::stringstream ss;
    ss << short_name_ << "-av:" << getAve() << "," << short_name_ << "-mx:" << getMax() << "," << short_name_
       << "-mn:" << getMin();
    return ss.str();
  }

  std::string getStr() override
  {
    std::stringstream ss;
    ss << long_name_ << " Ave: " << getAve() << "," << long_name_ << " Max: " << getMax() << "," << long_name_
       << " Min: " << getMin();
    return ss.str();
  }

  uint32_t getMax() const
  {
    return max_;
  }

  uint32_t getMaxMaxError() const
  {
    return max_max_error_;
  }

  uint32_t getMaxMaxWarn() const
  {
    return max_max_warn_;
  }

  uint32_t getMin() const
  {
    return min_;
  }

  uint32_t getMinMinError() const
  {
    return min_min_error_;
  }

  uint32_t getMinMinWarn() const
  {
    return min_min_warn_;
  }

  uint64_t getTotal() const
  {
    return total_;
  }

  void setMax(uint32_t max = 0)
  {
    max_ = max;
  }

  void setMaxMaxError(uint32_t maxMaxError = std::numeric_limits<uint32_t>::max())
  {
    max_max_error_ = maxMaxError;
  }

  void setMaxMaxWarn(uint32_t maxMaxWarn = std::numeric_limits<uint32_t>::max())
  {
    max_max_warn_ = maxMaxWarn;
  }

  void setMin(uint32_t min = std::numeric_limits<uint32_t>::max())
  {
    min_ = min;
  }

  void setMinMinError(uint32_t minMinError = 0)
  {
    min_min_error_ = minMinError;
  }

  void setMinMinWarn(uint32_t minMinWarn = 0)
  {
    min_min_warn_ = minMinWarn;
  }
};

};  // namespace am

#endif  // AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_AVE_H
