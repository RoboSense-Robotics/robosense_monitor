#ifndef RS_MONITOR_COMMON_SYSTEM_STATS_H
#define RS_MONITOR_COMMON_SYSTEM_STATS_H

#include <memory>
#include <string>
#include <vector>

namespace robosense::rs_monitor
{

struct SystemStats
{
  float total_cpu_usage{0.0f};
  std::vector<double> core_usage;
  uint32_t total_memory_kb{0};  // unit: KB
  uint32_t used_memory_kb{0};   // unit: KB
  float memory_usage{0.0f};
};

class SystemStatsCollector
{
public:
  virtual ~SystemStatsCollector() = default;
  virtual bool init() = 0;
  virtual bool collect(SystemStats & stats) = 0;

  static std::unique_ptr<SystemStatsCollector> create();
};

}  // namespace robosense::rs_monitor

#endif  // RS_MONITOR_COMMON_SYSTEM_STATS_H
