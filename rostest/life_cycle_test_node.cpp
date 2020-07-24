#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#include <super_lib/am_life_cycle.h>
#include <super_lib/am_stat.h>

namespace am
{

class AMNode : AMLifeCycle
{
private:
    class AMNodeStats
    {
        friend class AMNode;

        AMStat stat1 = AMStat("s1", "Stat 1", 1, 2);

    public:
        AMNodeStats(AMStatList &stat_list)
        {
            stat_list.add(&stat1);
        }
    };

    ros::NodeHandle nh_;
    ros::Subscriber stat1_sub_;
    ros::Subscriber throttle_sub_;
    AMNodeStats stats_;

public:

    AMNode() : nh_("~"), stats_(this->stats_list_)
    {
        stat1_sub_ = nh_.subscribe("stat1", 100, &AMNode::stat1CB, this);
        throttle_sub_ = nh_.subscribe("throttle", 100, &AMNode::throttleCB, this);
    }


    void stat1CB(const std_msgs::Int32::ConstPtr  &msg)
    {
        ROS_INFO_STREAM("got stat1 message: " << msg->data);
        stats_.stat1 = msg->data;
    }

    void throttleCB(const std_msgs::Float32::ConstPtr  &msg)
    {
        ROS_INFO_STREAM("got throttle message: " << msg->data);
        setThrottleS(msg->data);
    }
};

}; // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    am::AMNode node;

    ROS_INFO_STREAM(ros::this_node::getName() << " is running...");

    ros::spin();
}