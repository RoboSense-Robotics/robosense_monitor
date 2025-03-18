// Copyright 2025 RoboSense Technology Co., Ltd
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifdef _WIN32  // Windows implementation

#include <windows.h>
#include <psapi.h>
#include <pdh.h>
#include <pdhmsg.h>

#include "rs_monitor/common/system_stats.h"
#include "rs_monitor/common/system_utils.h"

namespace robosense::rs_monitor
{
class WindowsSystemStatsCollector : public SystemStatsCollector
{
public:
  WindowsSystemStatsCollector() = default;

  ~WindowsSystemStatsCollector()
  {
    if (cpu_query_) {
      PdhCloseQuery(cpu_query_);
    }
  }

  bool init() override
  {
    // 获取CPU核心数
    SYSTEM_INFO sys_info;
    GetSystemInfo(&sys_info);
    num_processors_ = sys_info.dwNumberOfProcessors;
    if (num_processors_ == 0) {
      return false;
    }

    // Initialize PDH query for CPU usage
    PdhOpenQuery(nullptr, 0, &cpu_query_);

    if (!cpu_query_) {
      return false;
    }

    PdhAddCounter(cpu_query_, "\\Processor(_Total)\\% Processor Time", 0, &cpu_total_counter_);

    if (!cpu_total_counter_) {
      return false;
    }

    // 为每个核心添加计数器
    cpu_core_counters_.resize(num_processors_);
    for (unsigned int i = 0; i < num_processors_; i++) {
      std::string counter_path = "\\Processor(" + std::to_string(i) + ")\\% Processor Time";
      PdhAddCounter(cpu_query_, counter_path.c_str(), 0, &cpu_core_counters_[i]);
    }

    PdhCollectQueryData(cpu_query_);
    return true;
  }

  bool collect(SystemStats & stats) override
  {
    return collect_cpu_stats(stats) && collect_memory_stats(stats);
  }

private:
  bool collect_cpu_stats(SystemStats & stats)
  {
    PDH_FMT_COUNTERVALUE counter_value;
    PdhCollectQueryData(cpu_query_);

    // 获取总体CPU使用率
    if (
      PdhGetFormattedCounterValue(cpu_total_counter_, PDH_FMT_DOUBLE, nullptr, &counter_value) ==
      ERROR_SUCCESS) {
      stats.total_cpu_usage = counter_value.doubleValue;
    } else {
      stats.total_cpu_usage = nan("0");
    }

    // 获取每个核心的CPU使用率
    stats.core_usage.clear();
    stats.core_usage.resize(num_processors_);

    for (unsigned int i = 0; i < num_processors_; i++) {
      if (
        PdhGetFormattedCounterValue(
          cpu_core_counters_[i], PDH_FMT_DOUBLE, nullptr, &counter_value) == ERROR_SUCCESS) {
        stats.core_usage[i] = counter_value.doubleValue;
      } else {
        stats.core_usage[i] = nan("0");
      }
    }

    return true;
  }

  bool collect_memory_stats(SystemStats & stats)
  {
    MEMORYSTATUSEX mem_status;
    mem_status.dwLength = sizeof(mem_status);
    if (GlobalMemoryStatusEx(&mem_status)) {
      stats.total_memory_kb = mem_status.ullTotalPhys / 1024;
      stats.used_memory_kb = (mem_status.ullTotalPhys - mem_status.ullAvailPhys) / 1024;
      stats.memory_usage = (stats.used_memory_kb * 100.0) / stats.total_memory_kb;
    } else {
      stats.total_memory_kb = 0;
      stats.used_memory_kb = 0;
      stats.memory_usage = nan("0");
    }

    return true;
  }

private:
  PDH_HQUERY cpu_query_{nullptr};
  PDH_HCOUNTER cpu_total_counter_{nullptr};
  std::vector<PDH_HCOUNTER> cpu_core_counters_;
  unsigned int num_processors_{0};
};

std::unique_ptr<SystemStatsCollector> SystemStatsCollector::create()
{
  return std::make_unique<WindowsSystemStatsCollector>();
}

}  // namespace robosense::rs_monitor

#endif
