#include <am_super/proc_stat.h>
#include <boost/algorithm/string/classification.hpp> // Include boost::for is_any_of
#include <boost/algorithm/string/split.hpp> // Include for boost::split

namespace am
{
	
ProcStat::CpuJiffies ProcStat::lineToJiffies(std::string line)
{
	std::vector<std::string> words;
	boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
	
	ProcStat::CpuJiffies results;
	results.line = line;
	results.prefix = words.at(0);
	results.user = std::stoi(words.at(1));
	results.nice = std::stoi(words.at(2));
	results.system = std::stoi(words.at(3));
	results.idle = std::stoi(words.at(4));
	results.iowait = std::stoi(words.at(5));
	results.irq = std::stoi(words.at(6));
	results.softirq = std::stoi(words.at(7));
	return results;
}

long ProcStat::totalJiffies(ProcStat::CpuJiffies jiffies)
{
	return jiffies.user + jiffies.nice + jiffies.system + jiffies.idle + jiffies.iowait + jiffies.irq + jiffies.softirq;
}

long ProcStat::workJiffies(ProcStat::CpuJiffies jiffies)
{
	return jiffies.user + jiffies.nice + jiffies.system;
}


int ProcStat::cpuUsage(ProcStat::CpuJiffies first, ProcStat::CpuJiffies second)
{
	float work= (float) ProcStat::workJiffies(second) -  ProcStat::workJiffies(first);
	float total= (float) ProcStat::totalJiffies(second) - ProcStat::totalJiffies(first);
	return work/total * 100 ;
}


}