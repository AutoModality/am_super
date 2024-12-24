#ifndef AM_LIDAR_BS_AM_LIDAR_BS_STATS_H_
#define AM_LIDAR_BS_AM_LIDAR_BS_STATS_H_

#include <super_lib/am_stat_list.h>
#include <super_lib/am_stat.h>
#include <super_lib/am_stat_reset.h>
#include <super_lib/am_stat_ave.h>
#include <super_lib/am_stat_status.h>

namespace am
{

class ResourceMonitorStats
{
public:
    AMStatStatus statStatus = AMStatStatus("ss", "AMStatStatus");


    AMStat tf_stats = AMStat("tf_s", "Transform Stats", 1, 2, 80, 99);
    AMStat node_stats = AMStat("n_s", "Nodes Stats", 1, 2, 80, 99);
    AMStat cpu_stats = AMStat("cpu_s", "CPU Stats", 1, 2, 80, 99);
    AMStat gpu_stats = AMStat("gpu_s", "GPU Stats", 1, 2, 80, 99);
    AMStat ram_stats = AMStat("ram_s", "RAM Stats", 1, 2, 80, 99);
    AMStat drive_stats = AMStat("drive_s", "Drive Stats", 1, 2, 80, 99);
    AMStat lidar_ip = AMStat("lidar_ip_s", "Lidar IP Stats", 1, 2, 80, 99);
    AMStat fl_ip = AMStat("fl_s", "FL IP Stats", 1, 2, 80, 99);
    AMStat fr_ip = AMStat("fr_s", "FR IP Stats", 1, 2, 80, 99);
    AMStat rl_ip = AMStat("rl_s", "RL IP Stats", 1, 2, 80, 99);
    AMStat rr_ip = AMStat("rr_s", "RR IP Stats", 1, 2, 80, 99);

    ResourceMonitorStats(AMStatList &stat_list)
    {
        stat_list.add(&statStatus);
        stat_list.add(&tf_stats);
        stat_list.add(&node_stats);
        stat_list.add(&gpu_stats);
        stat_list.add(&cpu_stats);
        stat_list.add(&ram_stats);
        stat_list.add(&drive_stats);
        stat_list.add(&lidar_ip);
        stat_list.add(&fl_ip);
        stat_list.add(&fr_ip);
        stat_list.add(&rl_ip);
        stat_list.add(&rr_ip);
    }
};

}; // namespace

#endif /* AM_LIDAR_BS_AM_LIDAR_BS_STATS_H_ */
