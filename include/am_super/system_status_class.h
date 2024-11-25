#ifndef AM_SUPER_INCLUDE_SYSTEM_STATUS_CLASS_H_
#define AM_SUPER_INCLUDE_SYSTEM_STATUS_CLASS_H_

#include <iostream>
#include <am_utils/am_ros2_utility.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unistd.h>

namespace am
{

struct MemoryInfo 
{
    unsigned long total;
    unsigned long free;
    unsigned long used;
    unsigned long available;
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

class SystemStatus
{
public:
    SystemStatus();

    ~SystemStatus();

    am::MemoryInfo& getMemoryInfo();

    am::CpuInfo getCPUInfo();

    void getGPUInfo(std::vector<am::GpuInfo> &gpu_infos);

    void getCPUInfo(std::vector<am::CpuInfo> &infos);

    double calculateCpuLoad(const am::CpuInfo &ci, const am::CpuInfo &ci_old);

    double getUpTime();

    void updateInfos();

    void print();

private:
    
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
};
}

#endif /*AM_SUPER_INCLUDE_SYSTEM_STATUS_CLASS_H_*/