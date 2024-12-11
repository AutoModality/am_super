#include <am_utils/am_ros2_utility.h>
#include <super_lib/am_life_cycle.h>
#include <resource_monitor/resource_status_class.h>
#include <resource_monitor/resource_monitor_node.h>

namespace am 
{

ResourceMonitorNode::ResourceMonitorNode(const std::string & node_name) : AMLifeCycle(node_name)
{
}

ResourceMonitorNode::~ResourceMonitorNode()
{

}

void ResourceMonitorNode::setAMClass(std::shared_ptr<ResourceStatus> am_class)
{
    resource_status_= am_class;
}

std::shared_ptr<am::ResourceStatus> ResourceMonitorNode::getAMClass()
{
    return resource_status_;
}

bool ResourceMonitorNode::onConfigure()
{    
    if(configured_)
    {
        return AMLifeCycle::onConfigure();
    }

    ROS_INFO("onConfigure");

    if(!resource_status_->onConfigure())
    {
        ROS_WARN("am_class_->onConfigure() failed");
        resource_status_->onCleanup();
        return false;
    }
    else
    {
        configured_ = true;
        return AMLifeCycle::onConfigure();
    }
}

bool ResourceMonitorNode::onCleanup()
{
    ROS_INFO("onCleanup");
    
    resource_status_->onCleanup();
    return AMLifeCycle::onCleanup();
}

void ResourceMonitorNode::heartbeatCB()
{
    resource_status_->heartbeatCB();
    AMLifeCycle::heartbeatCB();
}


} // namespace