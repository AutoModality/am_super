#ifndef AM_SUPER_INCLUDE_AM_SUPER_BABY_SITTER_H_
#define AM_SUPER_INCLUDE_AM_SUPER_BABY_SITTER_H_

#include <super_lib/am_life_cycle.h>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include <brain_box_msgs/msg/baby_sitter_status.hpp>
#include <brain_box_msgs/msg/node_status.hpp>
#include <vb_util_lib/bag_logger.h>
#include <vb_util_lib/trace.h>
#include <am_utils/am_ros2_utility.h>

namespace am
{
template <typename M>
class BabySitter
{
private:
  const int LOG_PERIOD_S = 10;
  const int NSECS_IN_MSECS = 1000000;
  const int MSECS_IN_SECS = 1000;

  enum DeviceState
  {
    OK,
    WARN,
    ERROR
  };

  using ErrorCB = std::function<void()>;  // @suppress("Type cannot be resolved") // @suppress("Symbol is not resolved")

  int min_ms_;
  int max_ms_;
  int ave_ms_;
  int freq_hz_;
  int warn_ms_;   // anything above this will increase the error count
  int error_ms_;  // if any one message takes longer than this, immediately return error
  int warn_count_;
  int warn_count_thresh_;  // number of warnings before error (one good resets this to 0)
  bool first_contact_;
  long last_contact_ms_;
  int timeout_ms_;
  int message_count_;
  int curr_min_ms_;
  int curr_max_ms_;
  std::string node_name_;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::GenericSubscription::SharedPtr device_data_sub_;
  rclcpp::Publisher<brain_box_msgs::msg::NodeStatus>::SharedPtr node_status_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  BagLogger* logger_;

  //	ErrorCB error_cb_;

  LifeCycleState node_state_;
  DeviceState device_state_;
  long start_time_ms_;
  long start_delay_ms_;

public:
  /*
   *	Constructor with ros::NodeHandle
   */
  BabySitter(rclcpp::Node::SharedPtr nh, BagLogger* logger, const std::string& node_name, const std::string& topic,
             int warn_ms, int error_ms, int warn_count_thresh = 5, int timeout_ms = 2000,
             int start_delay_ms = 0 /*, ErrorCB error_cb */);

  /*
   *	Destructor
   */
  ~BabySitter();

  /*
   *	these retur the data form the previous timer cycle
   */
  int getFreqHz();
  int getMinLatencyMs();
  int getMaxLatencyMs();
  int getAveLatencyMs();

private:
  void deviceCB(std::shared_ptr<rclcpp::SerializedMessage> msg);
  void heartbeatCB();
  void checkNodeState();
  void setNodeState(LifeCycleState node_state);
  std::string parseNodeState(LifeCycleState state);
  std::string parseDeviceState(DeviceState state);
  void printStatus();
  long nowMS();
};

template <class M>
BabySitter<M>::BabySitter(const rclcpp::Node::SharedPtr nh, BagLogger* logger, const std::string& node_name,
                          const std::string& topic, int warn_ms, int error_ms, int warn_count_thresh, int timeout_ms,
                          int start_delay_ms): nh_(nh)
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), nh_->get_name() << node_name);

  nh_ = nh;

  std::string parm = "~" + node_name + "/warn_ms";

  am::getParam<int>(nh_, parm, warn_ms_, warn_ms);
  RCLCPP_INFO_STREAM(nh_->get_logger(), nh_->get_name() << parm << " = " << warn_ms_);

  parm = "~" + node_name + "/error_ms";
  am::getParam<int>(nh_,parm, error_ms_, error_ms);
  RCLCPP_INFO_STREAM(nh_->get_logger(), nh_->get_name() << parm << " = " << error_ms_);

  parm = "~" + node_name + "/warn_count_thresh";
  am::getParam<int>(nh_,parm, warn_count_thresh_, warn_count_thresh);
  RCLCPP_INFO_STREAM(nh_->get_logger(), nh_->get_name() << parm << " = " << warn_count_thresh_);

  parm = "~" + node_name + "/timeout_ms";
  am::getParam<int>(nh_,parm, timeout_ms_, timeout_ms);
  RCLCPP_INFO_STREAM(nh_->get_logger(), nh_->get_name() << parm << " = " << timeout_ms_);

  min_ms_ = 1000;
  max_ms_ = 0;
  ave_ms_ = 0;
  freq_hz_ = 0;
  warn_count_ = 0;
  first_contact_ = true;
  last_contact_ms_ = 0;
  message_count_ = 0;
  curr_min_ms_ = 1000;
  curr_max_ms_ = 0;
  node_name_ = node_name;
  // error_cb_ = error_cb;
  node_state_ = LifeCycleState::INACTIVE;
  device_state_ = DeviceState::ERROR;
  checkNodeState();
  start_time_ms_ = nowMS();
  start_delay_ms_ = start_delay_ms;

  logger_ = logger;

  node_status_pub_ = nh_->create_publisher<brain_box_msgs::msg::NodeStatus>("/process/status", 1000);

  device_data_sub_ = nh_->create_generic_subscription(topic, am::getMessageName<M>(), 10, std::bind(&BabySitter<M>::deviceCB, this, std::placeholders::_1));

  heartbeat_timer_ = nh_->create_wall_timer(am::toDuration(1.0), std::bind(&BabySitter::heartbeatCB, this));
}

template <class M>
BabySitter<M>::~BabySitter()
{
}

template <class M>
int BabySitter<M>::getFreqHz()
{
  return freq_hz_;
}

template <class M>
int BabySitter<M>::getMinLatencyMs()
{
  return min_ms_;
}

template <class M>
int BabySitter<M>::getMaxLatencyMs()
{
  return max_ms_;
}

template <class M>
int BabySitter<M>::getAveLatencyMs()
{
  return ave_ms_;
}

template <class M>
std::string BabySitter<M>::parseNodeState(LifeCycleState state)
{
  switch (state)
  {
    case LifeCycleState::INACTIVE:
      return "INACTIVE";
      break;
    case LifeCycleState::ACTIVE:
      return "ACTIVE";
      break;
    default:
      return "UNKNOWN";
      break;
  }
}

template <class M>
std::string BabySitter<M>::parseDeviceState(DeviceState state)
{
  switch (state)
  {
    case DeviceState::OK:
      return "OK";
      break;
    case DeviceState::WARN:
      return "WARN";
      break;
    case DeviceState::ERROR:
      return "ERROR";
      break;
    default:
      return "UNKNOWN";
      break;
  }
}

template <class M>
void BabySitter<M>::printStatus()
{
	RCLCPP_INFO_STREAM(nh_->get_logger(), nh_->get_name() << node_name_ << ", node state:" << parseNodeState(node_state_)
                            << ", device state: " << parseDeviceState(device_state_));
}

template <class M>
void BabySitter<M>::checkNodeState()
{
  switch (node_state_)
  {
    case LifeCycleState::INACTIVE:
      if (device_state_ != DeviceState::ERROR)
      {
        setNodeState(LifeCycleState::ACTIVE);
      }
      break;
    case LifeCycleState::ACTIVE:
      if (device_state_ == DeviceState::ERROR)
      {
        setNodeState(LifeCycleState::INACTIVE);
      }
      break;
    default:
      RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 10, nh_->get_name() << node_name_ << ": unknown node state: " << parseNodeState(node_state_));
      break;
  }
}

template <class M>
void BabySitter<M>::setNodeState(LifeCycleState node_state)
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), nh_->get_name() << node_name_ << ": changing state from: " << parseNodeState(node_state_)
                            << " to: " << parseNodeState(node_state));

  switch (node_state_)
  {
    case LifeCycleState::INACTIVE:
      node_state_ = node_state;
      break;
    case LifeCycleState::ACTIVE:
      node_state_ = node_state;
      break;
    default:
      RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 10, nh_->get_name() << node_name_ << ": unknown node state: " << parseNodeState(node_state_));
      break;
  }
  printStatus();
}

template <class M>
void BabySitter<M>::deviceCB(std::shared_ptr<rclcpp::SerializedMessage> msg)
{
  message_count_++;
  long now_ms = nowMS();
  if (now_ms - start_time_ms_ < start_delay_ms_)
  {
    RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1.0, nh_->get_name() << node_name_ << ":message received during start delay");
  }

  long latency_ms = now_ms - last_contact_ms_;
  if (first_contact_)
  {
    last_contact_ms_ = now_ms;
    first_contact_ = false;
    return;
  }
  if (latency_ms > curr_max_ms_)
  {
    curr_max_ms_ = latency_ms;
  }
  if (latency_ms < curr_min_ms_)
  {
	  curr_min_ms_ = latency_ms;
  }
  if (latency_ms >= error_ms_)
  {
	  RCLCPP_ERROR_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1.0, nh_->get_name()<< node_name_ << ": max latency error: " << latency_ms << "(" << error_ms_
			  << ")");
	  device_state_ = DeviceState::ERROR;
  }
  else if (latency_ms >= warn_ms_)
  {
	  RCLCPP_WARN_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1.0, nh_->get_name() << node_name_ << ": latency warning: " << latency_ms << "(" << warn_ms_
			  << ")");
	  device_state_ = DeviceState::WARN;
	  warn_count_++;
	  if (warn_count_ >= warn_count_thresh_)
	  {
		  RCLCPP_ERROR_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1.0, nh_->get_name() << node_name_ << ": count latency error: " << warn_count_ << "("
				  << warn_count_thresh_ << ")");
		  device_state_ = DeviceState::ERROR;
	  }
  }
  else
  {
	  if (device_state_ != DeviceState::OK)
	  {
		  RCLCPP_INFO_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), 1.0, nh_->get_name() << node_name_ << ": latency ok: " << latency_ms);
		  device_state_ = DeviceState::OK;
	  }
	  warn_count_ = 0;
  }

  last_contact_ms_ = now_ms;
  checkNodeState();
}

template <class M>
long BabySitter<M>::nowMS()
{
  rclcpp::Time now = nh_->now();
  long now_ms = (long)(now.seconds() / NSECS_IN_MSECS) + (long)now.seconds() * MSECS_IN_SECS;
  return now_ms;
}

template <class M>
void BabySitter<M>::heartbeatCB()
{
  min_ms_ = curr_min_ms_;
  curr_min_ms_ = 1000;
  max_ms_ = curr_max_ms_;
  curr_max_ms_ = 0;
  if (message_count_)
  {
    ave_ms_ = 1000 / message_count_;
  }
  else
  {
    ave_ms_ = 1000;
  }
  freq_hz_ = message_count_;
  message_count_ = 0;

  brain_box_msgs::msg::BabySitterStatus log_msg;
  log_msg.name = node_name_;
  log_msg.freq = freq_hz_;
  log_msg.max_min_ave.max = max_ms_;
  log_msg.max_min_ave.min = min_ms_;
  log_msg.max_min_ave.ave = ave_ms_;
  LOG_MSG(log_msg, std::string("/status/super/" + node_name_), "brain_box_msgs/msg/BabySitterStatus", nh_->now(), 1);

  if (node_state_ == LifeCycleState::ACTIVE)
  {
    int time_since_contact = nowMS() - last_contact_ms_;
    if (time_since_contact > timeout_ms_)
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(), nh_->get_name() << node_name_ << ": timed out");
      device_state_ = DeviceState::ERROR;
      checkNodeState();
    }
  }

  if (node_state_ == LifeCycleState::ACTIVE)
  {
    brain_box_msgs::msg::NodeStatus ns_msg;
    ns_msg.node_name = node_name_;
    ns_msg.status = "ALIVE";
    ns_msg.value = "";
    ns_msg.process_id = 0;
    node_status_pub_->publish(ns_msg);
  }

  RCLCPP_INFO_STREAM_THROTTLE(nh_->get_logger(), *nh_->get_clock(), LOG_PERIOD_S, nh_->get_name() << node_name_ << " node:" << parseNodeState(node_state_)
                                                   << ", state: " << parseDeviceState(device_state_)
                                                   << ", max:" << max_ms_ << ", min: " << min_ms_
                                                   << ", ave: " << ave_ms_ << ", freq: " << freq_hz_);
}

}  // namespace am

#endif /* VISBOX_PACKAGES_SYSTEM_SUPER_INCLUDE_SUPER_BABYSITTER_H_	 */
