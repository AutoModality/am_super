#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_LIST_H
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT_LIST_H

#include <vector>

#include <diagnostic_updater/diagnostic_updater.h>
#include <super_lib/am_stat.h>

namespace am
{
/**
 * Specialized collection assisting with managing multiple stats simultaneously.
 */
class AMStatList
{
protected:
  std::vector<AMStat*> stat_list_;

public:
  AMStatList()
  {
  }

  void add(AMStat* stat)
  {
    stat_list_.push_back(stat);
  }

  LifeCycleStatus process(double warn_throttle_s, double error_throttle_s)
  {
    LifeCycleStatus status = LifeCycleStatus::OK;

    for (AMStat* stat : stat_list_)
    {
      AMStat::compoundStatus(status, stat->process(warn_throttle_s, error_throttle_s));
    }

    return status;
  }

  void reset()
  {
    for (AMStat* stat : stat_list_)
    {
      stat->reset();
    }
  }

  void addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw)
  {
    for (AMStat* stat : stat_list_)
    {
      stat->addStatistics(dsw);
    }
  }

  std::string getStatsStrShort()
  {
    std::stringstream ss;
    for (AMStat* stat : stat_list_)
    {
      ss << stat->getStrShort() << ",";
    }
    return ss.str();
  }

  std::string getStatsStr()
  {
    std::stringstream ss;
    for (AMStat* stat : stat_list_)
    {
      ss << stat->getStr() << ", ";
    }
    return ss.str();
  }
};

};  // namespace am

#endif  // AM_SUPER_INCLUDE_SUPER_LIB_AM_STAT__LIST_H
