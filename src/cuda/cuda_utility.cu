#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda/cuda_utility.h>


int getNumOfCudaDevices()
{
	int count;
	cudaGetDeviceCount(&count);
	return count;
}

CudaDevice getDeviceProperties(int count)
{
	CudaDevice d;
	cudaDeviceProp prop;
	cudaGetDeviceProperties(&prop, count);
	
	
	d.maxThreadsPerBlock = prop.maxThreadsPerBlock;
	d.totalGlobalMem = prop.totalGlobalMem;
	memcpy(d.name, prop.name, 256);
	d.memoryBusWidth = prop.memoryBusWidth;
	d.totalGlobalMem = prop.totalGlobalMem;
	d.sharedMemPerBlock = prop.sharedMemPerBlock;	
	cudaMemGetInfo(&d.freeMemory, &d.totalMemory);
	cudaDriverGetVersion(&d.driverVersion);
	cudaRuntimeGetVersion(&d.runtimeVersion);
	

	return d;
}