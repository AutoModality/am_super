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

    ResourceMonitorStats(AMStatList &stat_list)
    {
        stat_list.add(&statStatus);
        stat_list.add(&tf_stats);
        stat_list.add(&node_stats);
        stat_list.add(&gpu_stats);
        stat_list.add(&cpu_stats);
        stat_list.add(&ram_stats);
    }
};

}; // namespace

#endif /* AM_LIDAR_BS_AM_LIDAR_BS_STATS_H_ */
