#include <cmath>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "rs_monitor/common/common.h"
#include "rs_monitor/common/system_stats.h"

#ifdef __linux__

namespace robosense::rs_monitor
{

class LinuxSystemStatsCollector : public SystemStatsCollector
{
public:
  bool init() override { return true; }

  bool collect(SystemStats & stats) override
  {
    return collect_cpu_stats(stats) && collect_memory_stats(stats);
  }

private:
  bool collect_cpu_stats(SystemStats & stats)
  {
    std::ifstream stat_file("/proc/stat");
    if (!stat_file) {
      return false;
    }

    std::string line;
    if (!std::getline(stat_file, line)) {
      return false;
    }

    try {
      stats.total_cpu_usage = parse_cpu_stats(line, total_last_values_);
      stats.core_usage.clear();

      while (std::getline(stat_file, line)) {
        if (line.find("cpu") != 0) break;

        size_t core_id = stats.core_usage.size();
        if (core_last_values_.size() <= core_id) {
          core_last_values_.resize(core_id + 1);
        }
        stats.core_usage.push_back(parse_cpu_stats(line, core_last_values_[core_id]));
      }
    } catch (const std::exception &) {
      return false;
    }

    return true;
  }

  bool collect_memory_stats(SystemStats & stats)
  {
    std::ifstream meminfo("/proc/meminfo");
    if (!meminfo) {
      return false;
    }

    uint64_t total_mem = 0, free_mem = 0, buffers = 0, cached = 0;
    std::string line;

    try {
      while (std::getline(meminfo, line)) {
        if (line.find("MemTotal:") != std::string::npos)
          total_mem = parse_uint64_in_string(line);
        else if (line.find("MemFree:") != std::string::npos)
          free_mem = parse_uint64_in_string(line);
        else if (line.find("Buffers:") != std::string::npos)
          buffers = parse_uint64_in_string(line);
        else if (
          line.find("Cached:") != std::string::npos &&
          line.find("SwapCached:") == std::string::npos)
          cached = parse_uint64_in_string(line);
      }

      stats.total_memory_kb = total_mem;
      stats.used_memory_kb = total_mem - free_mem - buffers - cached;
      stats.memory_usage = total_mem > 0 ? (stats.used_memory_kb * 100.0) / total_mem : nan("0");
    } catch (const std::exception & e) {
      return false;
    }

    return true;
  }

private:
  struct CPUValues
  {
    uint64_t user{0}, nice{0}, system{0}, idle{0}, iowait{0}, irq{0}, softirq{0}, steal{0};
  };

  double parse_cpu_stats(const std::string & line, CPUValues & last_values)
  {
    CPUValues current;
    std::istringstream iss(line);
    std::string cpu_label;

    iss >> cpu_label >> current.user >> current.nice >> current.system >> current.idle >>
      current.iowait >> current.irq >> current.softirq >> current.steal;

    uint64_t total_time = current.user + current.nice + current.system + current.idle +
                          current.iowait + current.irq + current.softirq + current.steal;
    uint64_t idle_total = current.idle + current.iowait;

    uint64_t total_diff =
      total_time - (last_values.user + last_values.nice + last_values.system + last_values.idle +
                    last_values.iowait + last_values.irq + last_values.softirq + last_values.steal);
    uint64_t idle_diff = idle_total - (last_values.idle + last_values.iowait);

    last_values = current;

    return total_diff > 0 ? 100.0 * (1.0 - static_cast<double>(idle_diff) / total_diff) : nan("0");
  }

  CPUValues total_last_values_;
  std::vector<CPUValues> core_last_values_;
};

std::unique_ptr<SystemStatsCollector> SystemStatsCollector::create()
{
  return std::make_unique<LinuxSystemStatsCollector>();
}

}  // namespace robosense::rs_monitor

#endif
