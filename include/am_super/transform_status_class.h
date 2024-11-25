#ifndef AM_SUPER_INCLUDE_TRANSFORM_STATUS_CLASS_H_
#define AM_SUPER_INCLUDE_TRANSFORM_STATUS_CLASS_H_

#include <vb_util_lib/transformer.h>

namespace am
{

class TransformStatus
{
public:
    
    TransformStatus();
    
    ~TransformStatus();


private:
    rclcpp::TimerBase::SharedPtr check_timer_;
    
    void checkTimerCB();
};

}

#endif /*AM_SUPER_INCLUDE_TRANSFORM_STATUS_CLASS_H_*/
