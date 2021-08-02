#ifndef AM_SUPER_INCLUDE_PROC_STAT_H_
#define AM_SUPER_INCLUDE_PROC_STAT_H_

#include <string>

namespace am
{

class ProcStat
{
public:
	/*
	* 
	*/
	ProcStat();

	/** A single CPU line parsed into their appropriate jiffies.
	 * 
	 * https://www.linuxhowtos.org/System/procstat.htm
	 */
	struct CpuJiffies
	{
		std::string prefix;
		int user;
		int nice;
		int system;
		int idle;
		int iowait;
		int irq;
		int softirq;
		std::string line;
	};
 	
	 /** produces the aggregate cpu line when given the results of /proc/stat content 
	  * This is always the first line, prefixed with 'cpu'
	 */
	static CpuJiffies lineToJiffies(std::string line);

	/** Given the entire content, this returns the line that starts with the given prefix.
	 * Empty string is returned if no match
	*/
	static std::string filterCpuJiffies(std::string prefix, std::string content);

	/** Percent of CPU used from the earlier first to the latter second */
	static int cpuUsage(CpuJiffies first, CpuJiffies second);

	/** @return sum of values in the CPU jiffies */
	static long totalJiffies(CpuJiffies jiffies);

	/** @return sum of user,nice,system values in CPU jiffies*/
	static long workJiffies(CpuJiffies jiffies);

private:
 
};


}  // namespace am

#endif /* AM_SUPER_INCLUDE_PROC_STAT_H_	 */
