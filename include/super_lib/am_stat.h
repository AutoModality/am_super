#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_H
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_H

#include <cstdint>
#include <limits>

#include <ros/ros.h>

#include <super_lib/am_life_cycle_types.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace am
{
class AMStat
{
protected:
  std::string short_name_ = "short";
  std::string long_name_ = "long";
  uint32_t value_ = 0;
  uint32_t max_warn_ = std::numeric_limits<uint32_t>::max();
  uint32_t max_error_ = std::numeric_limits<uint32_t>::max();

private:
  AMStat();

public:
  AMStat(const std::string& short_name, const std::string& long_name)
  {
    short_name_ = short_name;
    long_name_ = long_name;
  }

  AMStat(const std::string& short_name, const std::string& long_name, uint32_t max_warn, uint32_t max_error)
    : AMStat(short_name, long_name)
  {
    max_warn_ = max_warn;
    max_error_ = max_error;
  }

  virtual ~AMStat()
  {
  }

  virtual LifeCycleStatus process(double warn_throttle_s, double error_throttle_s)
  {
    LifeCycleStatus status = LifeCycleStatus::OK;

    if (value_ > max_error_)
    {
      ROS_ERROR_STREAM_THROTTLE(error_throttle_s, long_name_ << " exceeding max_error: " << value_
                                                             << " (max:" << max_error_ << ")");
      compoundStatus(status, LifeCycleStatus::ERROR);
    }
    else if (value_ > max_warn_)
    {
      ROS_WARN_STREAM_THROTTLE(warn_throttle_s, long_name_ << " exceeding max_warn: " << value_ << " (max:" << max_warn_
                                                           << ")");
      compoundStatus(status, LifeCycleStatus::WARN);
    }

    return status;
  }

  virtual void reset()
  {
  }

  virtual void addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw)
  {
    dsw.add(long_name_, value_);
  }

  virtual std::string getStrShort()
  {
    std::stringstream ss;
    ss << short_name_ << ":" << value_;
    return ss.str();
  }

  virtual std::string getStr()
  {
    std::stringstream ss;
    ss << long_name_ << ": " << value_;
    return ss.str();
  }

  virtual void add(uint32_t adder)
  {
    value_ += adder;
  }

  AMStat& operator++(int)
  {
    value_++;
    return *this;
  }

  AMStat& operator+=(int adder)
  {
    value_ += adder;
    return *this;
  }

  void operator=(uint32_t assignment)
  {
    value_ = assignment;
  }

  uint32_t getCount() const
  {
    return value_;
  }

  const std::string& getLongName() const
  {
    return long_name_;
  }

  uint32_t getMaxError() const
  {
    return max_error_;
  }

  void setMaxError(uint32_t maxError = std::numeric_limits<uint32_t>::max())
  {
    max_error_ = maxError;
  }

  uint32_t getMaxWarn() const
  {
    return max_warn_;
  }

  void setMaxWarn(uint32_t maxWarn = std::numeric_limits<uint32_t>::max())
  {
    max_warn_ = maxWarn;
  }

  const std::string& getShortName() const
  {
    return short_name_;
  }

  static void compoundStatus(LifeCycleStatus& status, const LifeCycleStatus new_status)
  {
    if (new_status == LifeCycleStatus::ERROR)
    {
      status = LifeCycleStatus::ERROR;
    }
    else if (new_status == LifeCycleStatus::WARN && status == LifeCycleStatus::OK)
    {
      status = LifeCycleStatus::WARN;
    }
  }
};

};  // namespace am

#endif  // AM_OUSTER_OUSTER_STATS_H
