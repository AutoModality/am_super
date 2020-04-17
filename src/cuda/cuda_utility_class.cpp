/*
 * cuda_utility.cpp
 *
 *  Created on: Apr 17, 2020
 *      Author: ubuntu
 */

#include <cuda/cuda_utility_class.h>

namespace am
{
CudaUtility::CudaUtility(ros::NodeHandle &nh)
{
	update_timer_ = nh.createTimer(ros::Duration(1.0), &CudaUtility::timerCB, this);
	update_timer_.start();
}
CudaUtility::~CudaUtility()
{
}

void CudaUtility::stop_update()
{
	update_timer_.stop();
}
void CudaUtility::start_update()
{
	update_timer_.start();
}
CudaDevice CudaUtility::getDeviceUpdate()
{
	return device_info_;
}

//Timer callback
void CudaUtility::timerCB(const ros::TimerEvent &event)
{
	device_info_ = getDeviceUpdate();
}
}


