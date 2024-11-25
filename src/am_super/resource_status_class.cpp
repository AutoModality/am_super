#include <am_super/resource_status_class.h>

namespace am
{
ResourceStatus::ResourceStatus()
{
    cpu_cnt_ =  getCPUCoresCount();
}   

ResourceStatus::~ResourceStatus()
{
    
}

int ResourceStatus::getCPUCoresCount()
{
    std::ifstream file("/proc/stat");
    if (!file.is_open()) {
        ROS_ERROR("Unable to open /proc/stat");
    }

    int coreCount = 0;
    std::string line;

    while (std::getline(file, line)) 
    {
        if (line.compare(0, 3, "cpu") == 0 && line[3] >= '0' && line[3] <= '9') 
        {
            coreCount++;
        }
    }

    file.close();
    return coreCount;
}

am::CpuInfo ResourceStatus::parseCpuLine(const std::string& line) 
{
    am::CpuInfo info = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::istringstream iss(line);
    std::string cpuLabel; // e.g., "cpu0", "cpu1", etc.
    iss >> cpuLabel >> info.user >> info.nice >> info.system >> info.idle
        >> info.iowait >> info.irq >> info.softirq >> info.steal;

    info.total = info.user + info.nice + info.system + info.idle +
                 info.iowait + info.irq + info.softirq + info.steal;
    return info;
}

void ResourceStatus::updateInfos()
{
    getMemoryInfo();
    if(cpu_cnt_ < 0)
    {
        cpu_cnt_ = getCPUCoresCount();
        if(cpu_cnt_ < 0)
        {
            ROS_ERROR("Cannot get CPU count");
            return;
        }
    }

    cpu_infos_.resize(cpu_cnt_);
    cpu_infos_old_.resize(cpu_cnt_);
    cpu_loads_.resize(cpu_cnt_);

    getCPUInfo(cpu_infos_);

    if(is_first_time_)
    {
        getCPUInfo(cpu_infos_old_);
        is_first_time_ = false;
    }

    for(int i = 0; i < cpu_infos_.size(); i++)
    {
        cpu_loads_[i] = calculateCpuLoad(cpu_infos_[i], cpu_infos_old_[i]);
    }

    uptime_seconds_ = getUpTime();

    cpu_infos_old_ = cpu_infos_;

    getGPUInfo(gpu_infos_);
}


am::MemoryInfo& ResourceStatus::getMemoryInfo()
{
    mi = {0, 0, 0};
    std::ifstream file("/proc/meminfo");
    if (!file.is_open()) {
        ROS_ERROR("Error: Unable to open /proc/meminfo");
        return mi;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        unsigned long value;
        std::string unit;

        iss >> key >> value >> unit;

        if (key == "MemTotal:") {
            mi.total = value; // in kB
        } else if (key == "MemFree:") {
            mi.free = value; // in kB
        } else if (key == "Buffers:" || key == "Cached:") {
            mi.used += value; // Add buffers and cached to used
        }else if(key == "MemAvailable:"){
            mi.available = value;
        }

    }

    // Calculate used memory
    mi.used = mi.total - mi.free;
    file.close();
    return mi;
}

void ResourceStatus::getGPUInfo(std::vector<am::GpuInfo> &gpu_infos)
{
    gpu_infos.clear();
    // Execute the nvidia-smi command and read the output directly
    const std::string command = "nvidia-smi --query-gpu=name,utilization.gpu,temperature.gpu,memory.used,memory.free --format=csv,nounits,noheader";
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) 
    {
        ROS_ERROR("Error: Unable to execute nvidia-smi. Ensure it's installed and available in PATH.");
        return;
    }

    char buffer[128];
    std::ostringstream result;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result << buffer;
    }
    pclose(pipe);

    // Parse the command output
    std::istringstream iss(result.str());
    std::string line;
    while (std::getline(iss, line)) 
    {

        //ROS_INFO(GREEN "%s" COLOR_RESET, line.c_str());
        std::istringstream lineStream(line);
        am::GpuInfo gpu_info;
        // Parse memory used and free values
        std::string gpuName;
        int gpuUtilization, gpuTemperature, memoryUsed, memoryFree;

        // Using ',' to split the values
        std::getline(lineStream, gpuName, ',');  // Get the GPU name

        // Extracting the other values (removing leading/trailing spaces)
        lineStream >> gpuUtilization;
        lineStream.ignore();  // Ignore the space after the utilization
        lineStream >> gpuTemperature;
        lineStream.ignore();  // Ignore the space after the temperature
        lineStream >> memoryUsed;
        lineStream.ignore();  // Ignore the space after the memory used
        lineStream >> memoryFree;

        gpu_info.gpu_name = gpuName;
        gpu_info.util_percent = gpuUtilization;
        gpu_info.temp = gpuTemperature;
        gpu_info.mem_free = memoryFree;
        gpu_info.mem_used = memoryUsed;

        gpu_infos.push_back(gpu_info);
    }
}


void ResourceStatus::getCPUInfo(std::vector<am::CpuInfo> &infos)
{
    std::ifstream file("/proc/stat");
    if (!file.is_open()) {
        ROS_ERROR("Error: Unable to open /proc/stat");
        return;
    }

    std::string line;
    int cnt = 0;
    while (std::getline(file, line))
    {
        if (line.find("cpu") == 0 && line.find("cpu ") != 0) 
        {
            // Only process lines like "cpu0", "cpu1", etc.
            infos[cnt] = parseCpuLine(line);
            cnt++;
        }
    }

    file.close();
}

am::CpuInfo ResourceStatus::getCPUInfo()
{
    if(cpu_cnt_ < 0)
    {
        ROS_ERROR("Cannot get CPU Core Count: %d", cpu_cnt_);
    }

    am::CpuInfo cpu_info = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::ifstream file("/proc/stat");
    if (!file.is_open()) {
        ROS_ERROR("Error: Unable to open /proc/stat");
        return cpu_info;
    }

    std::string line;
    if (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string cpu;
        iss >> cpu >> cpu_info.user >> cpu_info.nice >> cpu_info.system
            >> cpu_info.idle >> cpu_info.iowait >> cpu_info.irq
            >> cpu_info.softirq >> cpu_info.steal;

        // Total CPU time
        cpu_info.total = cpu_info.user + cpu_info.nice + cpu_info.system +
                        cpu_info.idle + cpu_info.iowait + cpu_info.irq +
                        cpu_info.softirq + cpu_info.steal;
    }

    file.close();
    return cpu_info;
}

double ResourceStatus::calculateCpuLoad(const am::CpuInfo &ci, const am::CpuInfo &ci_old)
{
    unsigned long long totalDiff = ci.total - ci_old.total;
    unsigned long long idleDiff = (ci.idle + ci.iowait) - (ci_old.idle + ci_old.iowait);

    cpu_usage_ = (totalDiff - idleDiff) * 100.0 / totalDiff;

    return cpu_usage_;
}

double ResourceStatus::getUpTime()
{
    std::ifstream file("/proc/uptime");
    if (!file.is_open()) {
        ROS_INFO("Error: Unable to open /proc/uptime");

        return 0.0;
    }

    double idleSeconds;
    file >> uptime_seconds_ >> idleSeconds;
    file.close();

    return uptime_seconds_;
}

void ResourceStatus::print()
{
    ROS_INFO("MemoryInfo---> Total: %ld MB, Free: %ld MB, Used: %ld MB, Available: %ld MB", (mi.total / 1024), (mi.free / 1024), (mi.used / 1024), (mi.available / 1024));

    std::string msg = "";
    for(int i = 0; i < cpu_loads_.size(); i++)
    {
        msg += std::string(" Core[") + std::to_string(i) + "] Usage: " + std::to_string(cpu_loads_[i]);
    }

    ROS_INFO("CPUInfo---> Cores: %d , %s", cpu_cnt_, msg.c_str());

    ROS_INFO("UpTime: %f", uptime_seconds_);

    msg = "";
    for(int i = 0; i < gpu_infos_.size(); i++)
    {
        msg += gpu_infos_[i].gpu_name + ": Temp[C] = " + std::to_string(gpu_infos_[i].temp) + ", Used[%]: " + std::to_string(gpu_infos_[i].util_percent);
    }

    ROS_INFO("%s", msg.c_str());
}
}