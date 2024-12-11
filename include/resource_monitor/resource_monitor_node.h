#ifndef AM_SUPER_INCLUDE_RESOURCE_MONITOR_RESOURCE_MONITOR_NODE_H_
#define AM_SUPER_INCLUDE_RESOURCE_MONITOR_RESOURCE_MONITOR_NODE_H_


#include <resource_monitor/resource_status_class.h>

namespace am
{
class ResourceMonitorNode : public AMLifeCycle
{
public:
    ResourceMonitorNode(const std::string &node_name);

    ~ResourceMonitorNode();

    std::shared_ptr<am::ResourceStatus> resource_status_ = nullptr;

    std::shared_ptr<am::ResourceStatus> getAMClass();

    void setAMClass(std::shared_ptr<am::ResourceStatus> am_class);
    
    bool configured_ = false;

    // AMLifeCycle overrides
    void heartbeatCB() override;
    bool onCleanup() override;
    bool onConfigure() override;
};
}

#endif /*AM_SUPER_INCLUDE_RESOURCE_MONITOR_RESOURCE_MONITOR_NODE_H_*/