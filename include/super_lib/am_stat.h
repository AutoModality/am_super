#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_H
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_CNT_H

#include <cstdint>
#include <limits>

#include <ros/ros.h>

#include <super_lib/am_life_cycle_types.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace am
{
/**
 * Additive statistic looking for max thresholds exceeded during the entire Life Cycle.
 * 
 * This is the base statistic that looks for max thresholds for warnings and errors compared to the value. 
 * The value is set or incremented using standard operators (=, +=, ++).
 *
 * See others:
 * 
 * - AMStatReset has min/max thresholds for frequency checking. 
 * - AMStatAve is the average of value over count.   
 * 
 */
class AMStat
{
protected:
  std::string short_name_ = "short";
  std::string long_name_ = "long";
  uint32_t value_ = 0;
  uint32_t max_warn_ = std::numeric_limits<uint32_t>::max();
  uint32_t max_error_ = std::numeric_limits<uint32_t>::max();
  uint32_t min_warn_ = 0;
  uint32_t min_error_ = 0;
  /**indicates if min values are assigned */
  bool validate_min_ = false;
  bool validate_max_ = false;
  bool sample_received_ = false;
  bool sample_required_ = false;
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
    setMaxWarn(max_warn);
    setMaxError(max_error);
  }

  AMStat(const std::string& short_name, const std::string& long_name, uint32_t min_error, uint32_t min_warn,
              uint32_t max_warn, uint32_t max_error)
      : AMStat(short_name, long_name,max_warn,max_error)
  {
    setMinError(min_error);
    setMinWarn(min_warn);
  } 

  virtual ~AMStat()
  {
  }

  virtual LifeCycleStatus process(double warn_throttle_s, double error_throttle_s)
  {
    LifeCycleStatus status = LifeCycleStatus::OK;
  
    if(!sample_required_ || sample_received_)
    {
      if(validate_max_)
      {
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
      }

      if(validate_min_)
      {
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
      }

      if(!validate_max_ && !validate_min_)
      {
          ROS_WARN_STREAM_THROTTLE(error_throttle_s, long_name_ << " lacks validation since min/max is not set");        
      }
    }
    else
    {
      //sample is required and not yet received
      status = LifeCycleStatus::ERROR;
      ROS_ERROR_STREAM_THROTTLE(error_throttle_s, long_name_ << " no samples received [NAQE] ");      
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
    sample_received_ = true;
  }

  AMStat& operator++(int)
  {
    value_++;
    sample_received_ = true;
    return *this;
  }

  AMStat& operator+=(int adder)
  {
    value_ += adder;
    sample_received_ = true;
    return *this;
  }

  AMStat& operator=(uint32_t assignment)
  {
    value_ = assignment;
    sample_received_ = true;
    return *this;
  }

  /** Confusing name for return the value.  Use getValue() instead*/
  [[deprecated]]
  uint32_t getCount() const
  {
    return value_;
  }

  /**The current value for the stat */
  uint32_t getValue() const
  {
    return value_;
  }

  const std::string& getLongName() const
  {
    return long_name_;
  }

  void setWarnError(uint32_t max_warn, uint32_t max_error)
  {
    setMaxWarn(max_warn);
    setMaxError(max_error);
  }

  uint32_t getMaxError() const
  {
    return max_error_;
  }

  void setMaxError(uint32_t max_error)
  {
    max_error_ = max_error;
    validate_max_=true;
  }

  uint32_t getMaxWarn() const
  {
    return max_warn_;
  }

  void setMaxWarn(uint32_t max_warn)
  {
    validate_max_=true;
    max_warn_ = max_warn;
  }

  void setWarnError(uint32_t min_error, uint32_t min_warn, uint32_t max_warn, uint32_t max_error)
  {
    setMinError(min_error);
    setMinWarn(min_warn);
    setWarnError(max_warn,max_error);
  }

  uint32_t getMinError() const
  {
    return min_error_;
  }

  void setMinError(uint32_t min_error)
  {
    validate_min_=true;
    min_error_ = min_error;
  }

  uint32_t getMinWarn() const
  {
    return min_warn_;
  }

  void setMinWarn(uint32_t min_warn)
  {
    validate_min_=true;
    min_warn_ = min_warn;
  }
  
  const std::string& getShortName() const
  {
    return short_name_;
  }


  /**indicates if max values are assigned */
  const bool isValidatingMax() const 
  {
    return validate_max_;
  }

  /**indicates if min values are assigned */
  const bool isValidatingMin() const
  {
    return validate_min_;
  }

  /**indicates if this stat has received a value since constructed.*/
  const bool isSampleReceived() const
  {
    return sample_received_;
  }

  /**indicates if no samples received results in an error*/
  const bool isSampleRequired() const
  {
    return sample_required_;
  }

  void setSampleRequired(bool sample_required)
  {
    sample_required_ = sample_required;
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
