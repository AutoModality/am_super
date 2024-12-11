

#include <resource_monitor/resource_status_class.h>
#include <cstdlib>
#include <cstdio>   // For popen and fgets
#include <memory>   // For std::unique_ptr
#include <regex>    // For std::regex
#include <boost/filesystem.hpp>

namespace am
{
ResourceStatus::ResourceStatus(std::shared_ptr<am::ResourceMonitorStats> stats) : stats_(stats)
{

    sys_rep_pub_ = am::Node::node->create_publisher<brain_box_msgs::msg::SystemReport>(system_status_topic_,1);

    log_ctrl_sub_ = am::Node::node->create_subscription<brain_box_msgs::msg::LogControl>("/ctrl/log_control", 1, std::bind(&ResourceStatus::logCtrlCB, this, std::placeholders::_1));

    level_ = BagLogger::OFF;
    
    transformer_ = std::make_shared<am::Transformer>();

    sub_nets_add_ = getInetAddresses();

    for(const std::string &ip : sub_nets_add_)
    {   
        //ROS_INFO("subnet: %s", ip.c_str());
        if(ip == "192.168.1.1")
        {
            ip_check_ = true;
            ROS_INFO("Resource Monitor: looking for sensors on 192.168.1.1");
        }
    }
    
    getParams();

    timer_ = am::Node::node->create_wall_timer(am::toDuration(1.0), std::bind(&ResourceStatus::timerCB, this));

    cpu_cnt_ =  getCPUCoresCount();
}   

ResourceStatus::~ResourceStatus()
{
    
}

void ResourceStatus::logCtrlCB(const brain_box_msgs::msg::LogControl::SharedPtr msg)
{
    if (msg->enable)
	  {
		  ROS_INFO_STREAM(am::Node::node->get_name() << ": start logging to RS, level " << (int)msg->level);
		  BagLogger::instance()->startLogging("RS", msg->level);
		  enabled_ = true;
		  switch (msg->level)
		  {
		  case 1:
			  level_ = BagLogger::NORM;
			  break;
		  case 2:
			  level_ = BagLogger::FINE;
			  break;
		  case 3:
			  level_ = BagLogger::EXTRA;
			  break;
		  case 4:
			  level_ = BagLogger::ALL;
			  break;
		  default:
			  level_ = BagLogger::OFF;
			  break;
		  }
	  }
	  else
	  {
		  ROS_INFO_STREAM(am::Node::node->get_name() << ": stop logging");
		  BagLogger::instance()->stopLogging();
		  enabled_ = false;
		  level_ = BagLogger::OFF;

	  }
}

void ResourceStatus::getParams()
{

    //getting the ip sensor parameters
    int counter = 0;
    am::getParam<int>("ip_sensor_cnt", counter, counter);
    for(int i = 0; i < counter; i++)
    {
        std::string ip_check_str = "ip_sensor_" + std::to_string(i);
        std::string ip_address = "";
        std::string sensor_name = "";
        am::getParam<std::string>(ip_check_str + std::string(".ip_address") , ip_address, ip_address);
        am::getParam<std::string>(ip_check_str + std::string(".name") , sensor_name, sensor_name);
        if(ip_address == "" || sensor_name == "")
        {
            ROS_ERROR("ip sensor %d has configuration issues: ip: %s and name: %s", i, ip_address.c_str(), sensor_name.c_str());
            continue;
        }

        ip_addresses_[ip_address] = sensor_name;
        ROS_INFO(GREEN "IP Sensor[%s] is configured as %s" COLOR_RESET, ip_address.c_str(), sensor_name.c_str());
    }   

    //getting the transform list
    counter = 0;
    am::getParam<int>("transform_cnt", counter, counter);
    for(int i = 0; i < counter; i++)
    {
        std::string transform_str = "transform_" + std::to_string(i);
        std::string src = "";
        std::string target = "";
        am::getParam<std::string>(transform_str + std::string(".source") , src, src);
        am::getParam<std::string>(transform_str + std::string(".target") , target, target);

        if(src == "" || target == "")
        {
            ROS_ERROR("transform %d has configuration issues: source: %s and target: %s", i, src.c_str(), target.c_str());
            continue;
        }
        transform_list_.push_back(std::make_pair(src, target));
        ROS_INFO(GREEN "Transform check is set between source %s and target %s" COLOR_RESET, src.c_str(), target.c_str());
    }
}

std::shared_ptr<am::ResourceMonitorStats> ResourceStatus::getStats()
{
    return stats_;
}

bool ResourceStatus::onConfigure()
{
    status_sub_ = am::Node::node->create_subscription<std_msgs::msg::Int32>(std::string(am::Node::node->get_name()) + "/status", 100, std::bind(&ResourceStatus::statusCB, this, std::placeholders::_1));
    
    stat_sub_ = am::Node::node->create_subscription<std_msgs::msg::Int32>(std::string(am::Node::node->get_name()) + "/stat", 100, std::bind(&ResourceStatus::statCB, this, std::placeholders::_1));
    
    return true;
}

bool ResourceStatus::onCleanup()
{
    status_sub_.reset();
    stat_sub_.reset();
    return true;
}

void ResourceStatus::statusCB(const std_msgs::msg::Int32::SharedPtr msg)
{
    stats_->statStatus = msg->data;
}

void ResourceStatus::statCB(const std_msgs::msg::Int32::SharedPtr msg)
{

}

void ResourceStatus::heartbeatCB()
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

void ResourceStatus::updateInfos(brain_box_msgs::msg::SystemReport &sys_report)
{
    getMemoryInfo();
    sys_report.mem_free_percentage = (float)(100 - mi.used_percent);
    sys_report.mem_total_mb = (float)mi.total;
    sys_report.mem_used_percentage = (float)mi.used_percent;
    sys_report.cores_count = cpu_cnt_;
    if(cpu_cnt_ < 0)
    {
        cpu_cnt_ = getCPUCoresCount();
        if(cpu_cnt_ < 0)
        {
            sys_report.error_msg += ", Cannot get CPU count";
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
    double avg_load = 0.0;
    for(int i = 0; i < cpu_infos_.size(); i++)
    {
        double load = calculateCpuLoad(cpu_infos_[i], cpu_infos_old_[i]);
        avg_load+=load;
        cpu_loads_[i] = load;
        sys_report.cpu_percent_cores.push_back(load);
    }

    if(cpu_cnt_ > 0)
    {
        avg_load = avg_load/cpu_cnt_;
        sys_report.cpu_percent_total = avg_load;
        stats_->cpu_stats = (avg_load > 80.0?100:50);
    }
    

    uptime_seconds_ = getUpTime();
    sys_report.uptime_seconds = uptime_seconds_;

    cpu_infos_old_ = cpu_infos_;

    gpu_info_ = getGPUInfo();
    sys_report.gpu_used_percentage = (float)gpu_info_.load_percent;
    sys_report.gpu_temp = (float)gpu_info_.temp;
    stats_->gpu_stats = 50;
    //ROS_INFO("GPU Load Percent: %d , Temp: %d", gpu_info_.load_percent, gpu_info_.temp);
    if(gpu_info_.load_percent > 90)
    {
        stats_->gpu_stats = 100;
        ROS_ERROR("GPU Issues: LOAD Percent: %d, Temp: %d", gpu_info_.load_percent, gpu_info_.temp);
    }


    //check the drive stats
    stats_->drive_stats = 50;
    int coef = 1024*1024;
    am::DiskInfo disk_info = getDiskInfo();
    sys_report.hd_used_percentage = disk_info.percentUsed;
    sys_report.hd_used = (float)(disk_info.usedSpace/coef);
    if(disk_info.percentUsed > 98.0)
    {
        stats_->drive_stats = 100;
        
        ROS_ERROR("Disk total: %lld MB, available: %lld MB, used: %lld MB, percentage: %f", (disk_info.totalSpace/coef), (disk_info.availableSpace/coef), (disk_info.usedSpace/coef), disk_info.percentUsed);
    }
    
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
    mi.used_percent = (mi.used / mi.total) * 100;
    stats_->ram_stats = (mi.used_percent > 80?100:50);
    file.close();
    return mi;
}

// Function to read the content of a file
std::string ResourceStatus::readFile(const std::string& path) 
{
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("Error: Unable to open file " + path);
    }

    std::string content;
    std::getline(file, content);
    file.close();
    return content;
}

am::GpuInfo ResourceStatus::getGPUInfo()
{
    am::GpuInfo gpu_info;

    // "/sys/devices/gpu.0/load" exists only in Jetpack
    //in contrast, nvidia-smi only exists in amd64 architure
    const std::string loadPath = "/sys/devices/gpu.0/load";
    if (boost::filesystem::exists(loadPath))
    {
        std::string loadStr = readFile(loadPath);
        gpu_info.load_percent = std::stoi(loadStr) / 1000; // Convert to percentage

        const std::string baseThermalPath = "/sys/class/thermal/";
        const std::string typeSuffix = "/type";
        const std::string tempSuffix = "/temp";

        bool found_gpu_file = false;
        for (int i = 0; i < 10; ++i)
        { // Check up to 10 thermal zones
            try {
                    std::string typePath = baseThermalPath + "thermal_zone" + std::to_string(i) + typeSuffix;
                    //ROS_INFO("Type file: %s", typePath.c_str());
                    if(!boost::filesystem::exists(typePath))
                    {
                        continue;
                    }
                    std::string type = readFile(typePath);
                    if (type.find("GPU") != std::string::npos) 
                    { // Look for the GPU thermal zone
                        found_gpu_file = true;
                        std::string tempPath = baseThermalPath + "thermal_zone" + std::to_string(i) + tempSuffix;
                        std::string tempStr = readFile(tempPath);
                        gpu_info.temp = std::stoi(tempStr) / 1000; // Convert millidegrees to degrees Celsius
                        break;
                    }
            } catch (...) 
            {
                // Ignore errors and continue checking other zones
            }
        }

        return gpu_info;
    }


    // Execute the nvidia-smi command and read the output directly
    const std::string command = "nvidia-smi --query-gpu=name,utilization.gpu,temperature.gpu,memory.used,memory.free --format=csv,nounits,noheader";
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) 
    {
        ROS_ERROR("Error: Unable to execute nvidia-smi. Ensure it's installed and available in PATH.");
        return gpu_info;
    }

    char buffer[128];
    std::ostringstream result;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) 
    {
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
        gpu_info.load_percent = gpuUtilization;
        gpu_info.temp = gpuTemperature;
        
        return gpu_info;
    }


    
    return gpu_info;
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

bool ResourceStatus::isReachable(const std::string &ip_address)
{
    std::string command = std::string("ping -c 1 -W 0.2 ") + ip_address + std::string(" >/dev/null 2>&1");

    int result = std::system(command.c_str());

    return result == 0;
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
    msg += "GPU: Temp[C] = " + std::to_string(gpu_info_.temp) + ", Used[%]: " + std::to_string(gpu_info_.load_percent);

    ROS_INFO("%s", msg.c_str());
}

void ResourceStatus::checkNodeNames(brain_box_msgs::msg::SystemReport &sys_report)
{
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph = am::Node::node->get_node_graph_interface();

    std::vector<std::string> running_nodes = node_graph->get_node_names();
    sys_report.nodes = running_nodes;

    std::unordered_map<std::string, int> string_count;

    // Count occurrences of each string
    for (const std::string& str : running_nodes) 
    {
        if(str.find("plugin_name") != std::string::npos)
        {
            continue;
        }
        string_count[str]++;
    }

    // Collect strings that appear more than once
    std::string error_msg = "";
    bool node_check = true;
    for (const auto& [str, count] : string_count) 
    {
        if (count > 1) 
        {
            error_msg += " Found a duplicate Node: " + str;
            ROS_ERROR("%s", error_msg.c_str());
            sys_report.error_msg += error_msg;
        }
    }
    stats_->node_stats = (node_check?50:100);
}

void ResourceStatus::checkTransforms(brain_box_msgs::msg::SystemReport &sys_report)
{
    bool tf_check = true;
    for(std::pair<std::string, std::string> &tf_str : transform_list_)
    {
        geometry_msgs::msg::TransformStamped transform;
        brain_box_msgs::msg::TransformStatus tf_status_msg;
        tf_status_msg.source = tf_str.first;
        tf_status_msg.target = tf_str.second;
        tf_status_msg.status = brain_box_msgs::msg::TransformStatus::CONNECTED;
        if(!transformer_->getTransform(tf_str.first, tf_str.second, transform, 1.0, false))
        {
            std::string error_msg = std::string(", Transform tree is broken between: ") + tf_str.first + " and " + tf_str.second;
            //ROS_ERROR("Transform tree is broken: %s, %s", tf_str.first.c_str(), tf_str.second.c_str());
            tf_check = false;
            tf_status_msg.status = brain_box_msgs::msg::TransformStatus::DISCONNECTED;
            sys_report.error_msg += error_msg;
        }

        sys_report.tf_tree_status.push_back(tf_status_msg);
    }
    stats_->tf_stats = (tf_check?50:100);
}

void ResourceStatus::checkSensorIPs(brain_box_msgs::msg::SystemReport &sys_report)
{  

    
    //IP Address Check
    stats_->lidar_ip = 50;
    stats_->fl_ip = 50;
    stats_->fr_ip = 50;
    stats_->rl_ip = 50;
    stats_->rr_ip = 50;
    //Only if you have the subnet
    if(ip_check_)
    {
        std::map<std::string, std::string>::iterator it = ip_addresses_.begin();
        for(; it != ip_addresses_.end(); ++it)
        {
            brain_box_msgs::msg::IPSensorStatus ip_sensor_msg; 
            ip_sensor_msg.ip = it->first;
            ip_sensor_msg.alias= it->second;
            ip_sensor_msg.status = brain_box_msgs::msg::IPSensorStatus::CONNECTED;  
            if(!isReachable(it->first))
            {
                ip_sensor_msg.status = brain_box_msgs::msg::IPSensorStatus::DISCONNECTED;  
                std::string error_msg;
                //THE DEVICE CANNOT BE REACHED
                if(it->second == "lidar")
                {
                    
                    stats_->lidar_ip = 100;
                    error_msg = " Lidar is not reachable";
                    sys_report.error_msg += error_msg;
                    ROS_ERROR("%s", error_msg.c_str());
                }   
                if(it->second == "front_left")
                {
                    stats_->fl_ip = 100;
                    error_msg = " Front Left is not reachable";
                    sys_report.error_msg += error_msg;
                    ROS_ERROR("%s", error_msg.c_str());
                }      
                if(it->second == "front_right")
                {
                    stats_->fr_ip = 100;
                    error_msg = " Front Right is not reachable";
                    sys_report.error_msg += error_msg;
                    ROS_ERROR("%s", error_msg.c_str());
                }  
                if(it->second == "rear_right")
                {
                    stats_->rr_ip = 100;
                    error_msg = " Rear Right Left is not reachable";
                    sys_report.error_msg += error_msg;
                    ROS_ERROR("%s", error_msg.c_str());
                }      
                if(it->second == "rear_left")
                {
                    stats_->rl_ip = 100;
                    error_msg = " Rear Left is not reachable";
                    sys_report.error_msg += error_msg;
                    ROS_ERROR("%s", error_msg.c_str());
                }   
            }
            sys_report.ip_sensors.push_back(ip_sensor_msg);
        }
    }
}


// Function to execute the nmap command and capture the output
std::unordered_set<std::string> ResourceStatus::getActiveIPs(const std::string& subnet) 
{
    std::unordered_set<std::string> activeIPs;
    std::string command = "nmap -sn " + subnet;

    // Open a pipe to execute the command and read its output
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
        std::cerr << "Error: Failed to run nmap command.\n";
        return activeIPs;
    }

    // Read the command output line by line
    char buffer[128];
    std::string line;
    while (fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) 
    {
        line = buffer;
        // Check if the line contains "Nmap scan report for", indicating a live IP
        if (line.find("Nmap scan report for") != std::string::npos) 
        {
            std::string ip = line.substr(line.find_last_of(' ') + 1);
            ip.erase(ip.find('\n'));  // Remove the newline character
            activeIPs.insert(ip);
        }
    }

    return activeIPs;
}


// Function to execute ifconfig and extract inet addresses
std::vector<std::string> ResourceStatus::getInetAddresses() 
{
    std::vector<std::string> inetAddresses;
    std::string command = "ifconfig";
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
    
    if (!pipe) {
        std::cerr << "Error: Failed to run ifconfig command.\n";
        return inetAddresses;
    }

    char buffer[256];
    std::string output;

    // Read the entire output of ifconfig
    while (fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
        output += buffer;
    }

    // Regular expression to match inet (IPv4) addresses
    std::regex inetRegex(R"(inet\s+(\d+\.\d+\.\d+\.\d+))");
    std::smatch match;

    // Search for inet addresses in the output
    auto begin = output.cbegin();
    auto end = output.cend();
    while (std::regex_search(begin, end, match, inetRegex)) {
        inetAddresses.push_back(match[1]);
        begin = match.suffix().first;  // Move to the next match
    }

    return inetAddresses;
}


// Function to get disk usage information
DiskInfo ResourceStatus::getDiskInfo(const std::string& path) 
{
    struct statvfs stat;

    if (statvfs(path.c_str(), &stat) != 0) {
        throw std::runtime_error("Error: Unable to get disk information for " + path);
    }

    DiskInfo info;
    info.totalSpace = stat.f_blocks * stat.f_frsize;      // Total blocks * block size
    info.availableSpace = stat.f_bavail * stat.f_frsize;  // Available blocks * block size
    info.usedSpace = info.totalSpace - info.availableSpace;
    info.percentUsed = (info.usedSpace * 100.0) / info.totalSpace;

    return info;
}

/*
    Timer Callback: this is where everything is updated
 */
void ResourceStatus::timerCB()
{

    brain_box_msgs::msg::SystemReport sr_msg;
    sr_msg.header.stamp = am::ClockNow();

    //Checking the repeated node name
    checkNodeNames(sr_msg);

    //Transform check
    checkTransforms(sr_msg);

    //sensor ip check
    checkSensorIPs(sr_msg);
    
    //Resource Check
    updateInfos(sr_msg);

    sys_rep_pub_->publish(sr_msg);

    LOG_MSG(system_status_topic_, sr_msg, 2);
    
}
}