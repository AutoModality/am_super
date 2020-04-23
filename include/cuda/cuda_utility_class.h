/*
 * cuda_utility_class.h
 *
 *  Created on: Apr 17, 2020
 *      Author: ubuntu
 */

#ifndef AM_SUPER_INCLUDE_CUDA_CUDA_UTILITY_CLASS_H_
#define AM_SUPER_INCLUDE_CUDA_CUDA_UTILITY_CLASS_H_

#include <iostream>
#include <cuda/cuda_utility.h>
#include <ros/ros.h>

namespace am
{
class CudaUtility
{
public:
	CudaUtility(ros::NodeHandle &nh);
	~CudaUtility();

	void stop_update();
	void start_update();
	CudaDevice getDeviceUpdate();
	void display();
private:
	CudaDevice device_info_;
	ros::Timer update_timer_;
	void timerCB(const ros::TimerEvent &event);


};
}



#endif /* AM_SUPER_INCLUDE_CUDA_CUDA_UTILITY_CLASS_H_ */
