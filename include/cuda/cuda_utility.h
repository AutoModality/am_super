/*
 * cuda_utility.h
 *
 *  Created on: Apr 17, 2020
 *      Author: ubuntu
 */

#ifndef AM_SUPER_INCLUDE_CUDA_CUDA_UTILITY_H_
#define AM_SUPER_INCLUDE_CUDA_CUDA_UTILITY_H_

#include <stdio.h>

struct CudaDevice
{
  int maxThreadsPerBlock;
  int ECCEnabled;
  int asyncEngineCount;
  int canMapHostMemory;
  int canUseHostPointerForRegisteredMem;
  int clockRate;
  int computeMode;
  int concurrentKernels;
  int concurrentManagedAccess;
  int cooperativeLaunch;
  int memoryBusWidth;
  int driverVersion;
  int runtimeVersion;
  int deviceTemperature;
  char name[256];
  size_t sharedMemPerBlock;
  size_t totalGlobalMem;
  size_t freeMemory;
  size_t totalMemory;
};

// Utility Functions
int getNumOfCudaDevices();
CudaDevice getDeviceProperties(int count);

#endif /* AM_SUPER_INCLUDE_CUDA_CUDA_UTILITY_H_ */
