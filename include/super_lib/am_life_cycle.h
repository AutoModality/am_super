#ifndef AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_H_
#define AM_SUPER_INCLUDE_SUPER_LIB_AM_LIFECYCLE_H_

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticStatus.h>

#include <super_lib/am_stat_list.h>
#include <super_lib/am_life_cycle_types.h>

namespace am
{
class AMLifeCycle
{
private:
  LifeCycleState state_;
  LifeCycleStatus status_;

public:
  LifeCycleState getState() const;
  void setState(const LifeCycleState state);
  LifeCycleStatus getStatus() const;
  void setStatus(const LifeCycleStatus status);

protected:
  diagnostic_updater::Updater updater_;
  AMStatList stats_list_;

  ros::NodeHandle nh_;
  ros::Timer heartbeat_timer_;
  ros::Publisher state_pub_;

  /**
   * @brief Default constructor
   */
  AMLifeCycle();

  /**
   * @brief Virtual destructor
   */
  virtual ~AMLifeCycle();

  virtual void configure();
  virtual void cleanup();
  virtual void activate();
  virtual void deactivate();
  virtual void shutdown();
  virtual void destroy();

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from UNCONFIGURED to INACTIVE.
   */
  virtual void onConfigure();

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from INACTIVE to UNCONFIGURED.
   */
  virtual void onCleanUp();

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from INACTIVE to ACTIVE.
   */
  virtual void onActivate();

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from ACTIVE to INACTIVE.
   */
  virtual void onDeactivate();

  /**
   * @brief Function to be defined by the user.
   *        Called at the end of transition from INACTIVE to FINALIZED.
   */
  virtual void onShutdown();

  /**
   * @brief Function to be defined by the user.
   *        Called after an error and may transition to UNCONFIGURED or FINALIZED.
   */
  virtual void onError();

  virtual void addStatistics(diagnostic_updater::DiagnosticStatusWrapper& dsw);
  virtual void heartbeatCB(const ros::TimerEvent& event);

};  // class AMLifeCycle

};  // namespace am

#endif
