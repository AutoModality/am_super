#ifndef AM_SUPER_INCLUDE_RESOURCE_MONITOR_RESOURCE_STATUS_CLASS_H_
#define AM_SUPER_INCLUDE_RESOURCE_MONITOR_RESOURCE_STATUS_CLASS_H_

#include <iostream>
#include <am_utils/am_ros2_utility.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vb_util_lib/transformer.h>
#include <resource_monitor/resource_monitor_stats.h>
#include <std_msgs/msg/int32.hpp>


namespace am
{

struct MemoryInfo 
{
    unsigned long total;
    unsigned long free;
    unsigned long used;
    unsigned long available;
    int used_percent;
};

struct GpuInfo
{
    std::string gpu_name;
    int temp;
    int mem_used;
    int mem_free;
    int util_percent;
};

struct CpuInfo 
{
    unsigned long long user;
    unsigned long long nice;
    unsigned long long system;
    unsigned long long idle;
    unsigned long long iowait;
    unsigned long long irq;
    unsigned long long softirq;
    unsigned long long steal;
    unsigned long long total;
};

class ResourceStatus
{
public:
    ResourceStatus(std::shared_ptr<am::ResourceMonitorStats> stats);

    ~ResourceStatus();

    am::MemoryInfo& getMemoryInfo();

    am::CpuInfo getCPUInfo();

    void getGPUInfo(std::vector<am::GpuInfo> &gpu_infos);

    void getCPUInfo(std::vector<am::CpuInfo> &infos);

    double calculateCpuLoad(const am::CpuInfo &ci, const am::CpuInfo &ci_old);

    double getUpTime();

    void updateInfos();

    void print();

    bool isReachable(const std::string &ipAddress);

    void getParams();

    std::unordered_set<std::string> getActiveIPs(const std::string& subnet = "192.168.1.0/24");

    std::shared_ptr<am::ResourceMonitorStats> getStats();

    std::vector<std::string> getInetAddresses();

     // AMLifeCycle passthrus
    bool onConfigure();

    bool onCleanup();
    
    void heartbeatCB();

private:
    
    std::shared_ptr<am::ResourceMonitorStats> stats_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr status_sub_;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stat_sub_;

    void statusCB(const std_msgs::msg::Int32::SharedPtr msg);
    
    void statCB(const std_msgs::msg::Int32::SharedPtr msg);

    int getCPUCoresCount();

    am::CpuInfo parseCpuLine(const std::string &line);

    int cpu_cnt_= -1;

    double cpu_usage_;

    double uptime_seconds_;

    bool is_first_time_ {true};

    std::vector<double> cpu_loads_;

    am::MemoryInfo mi;

    std::vector<am::CpuInfo> cpu_infos_;

    std::vector<am::CpuInfo> cpu_infos_old_;

    std::vector<am::GpuInfo> gpu_infos_;

    std::map<std::string, std::string> ip_addresses_; //IPAddress, Name

    /*ROS Infrastructure Checking tools*/
    std::shared_ptr<am::Transformer> transformer_;

    std::vector<std::pair<std::string, std::string>> transform_list_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    void timerCB();

    void checkNodeNames();

    void checkTransforms();

    void checkSensorIPs();
};
}

#endif /*AM_SUPER_INCLUDE_RESOURCE_MONITOR_RESOURCE_STATUS_CLASS_H_*/