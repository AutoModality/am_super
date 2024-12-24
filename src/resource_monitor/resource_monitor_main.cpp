#include <am_utils/am_ros2_utility.h>

#include <resource_monitor/resource_monitor_stats.h>
#include <resource_monitor/resource_monitor_node.h>

std::shared_ptr<am::AMLifeCycle> am::Node::node = nullptr;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // create the AMLifeCycle object with stats and assign it to the AMNode singleton
    std::shared_ptr<am::ResourceMonitorNode> am_node = std::make_shared<am::ResourceMonitorNode>("resource_monitor");
    std::shared_ptr<am::ResourceMonitorStats> stats = std::make_shared<am::ResourceMonitorStats>(am_node->stats_list_);    
    am::Node::node = am_node;

    // create the buisness logic object and give the AMLifecycle class a pointer to it
    std::shared_ptr<am::ResourceStatus> am_class = std::make_shared<am::ResourceStatus>(stats);
    am_node->setAMClass(am_class);

    ROS_INFO_STREAM(am::Node::node->get_name() << ": running...");

    rclcpp::spin(am::Node::node);

    rclcpp::shutdown();

    return 0;
}